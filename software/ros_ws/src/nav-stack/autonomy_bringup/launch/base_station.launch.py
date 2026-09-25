"""Base station view: RViz and the point cloud decoders, for an operator machine
watching a robot elsewhere.

This launches the operator side alone, so it can be run on a laptop that shares a ROS
domain with the rover while the sensing, estimation and health monitoring all run on the
rover itself. The one thing here that can drive the robot is the joystick, which is
deliberate: it is the manual override for when autonomous navigation goes wrong. Plug the
controller into this machine and it is live -- joy_node picks the device up on hotplug.
teleop's generic_controller only publishes /joy_vel while the drive deadman is held, and
twist_mux on the rover ranks joy_vel above navigation, so holding the deadman takes over
from nav2 at once and releasing it hands control back after twist_mux's 0.5 s timeout.

point_cloud_decompress.launch.py rides along because the clouds arriving from the rover
are Draco-encoded and RViz cannot display them as they are -- see the comment on the
include below.

The default config docks the Topic Health panel from rviz_plugins, which tabulates
the rate, bandwidth and staleness of each monitored topic from the rover's health_monitor.
That table is the quickest way to tell a link problem from a node that has died.

Arguments:
    rviz_config   path to an RViz config, to open a different view without editing this file
    use_sim_time  set true when following a simulated robot, so displays honour /clock
    use_nixgl     wrap RViz in nixGL for GPU access; true matches the other launch files
                  in this repo, false runs rviz2 directly on a machine with working drivers
    decompress    run the Draco decoders; false when the rover is sending raw clouds, or
                  when another process on this machine already decodes them
    mesh          reconstruct a surface from the decoded map cloud, published as a marker
                  on /mesh for RViz. Off by default: it is only useful once the rover has
                  mapped something, and it costs a fraction of a second per update
    rviz_only     skip the point cloud decoders, mesh node and minimap, launching RViz
                  alone. Overrides decompress/mesh/minimap regardless of their own values.
                  The joystick is not affected
    teleop        run the joystick override (joy_node and teleop's generic_controller)
    controller_type  controller config for the override, as teleop's controller.launch.py
                  type:= -- taranis, xbox, logitech or 8bitdo
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_nixgl = LaunchConfiguration("use_nixgl")
    decompress = LaunchConfiguration("decompress")
    mesh = LaunchConfiguration("mesh")
    rviz_only = LaunchConfiguration("rviz_only")
    teleop = LaunchConfiguration("teleop")

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("autonomy_bringup"), "rviz", "base_station.rviz"]
        ),
        description="RViz config to open",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use /clock instead of wall time",
    )
    use_nixgl_arg = DeclareLaunchArgument(
        "use_nixgl",
        default_value="true",
        description="Wrap RViz in nixGL for GPU access",
    )
    decompress_arg = DeclareLaunchArgument(
        "decompress",
        default_value="true",
        description="Decode the rover's Draco point cloud topics back into PointCloud2",
    )
    rviz_only_arg = DeclareLaunchArgument(
        "rviz_only",
        default_value="false",
        description=(
            "Launch RViz alone, skipping the point cloud decoders, mesh node and "
            "minimap regardless of decompress/mesh/minimap"
        ),
    )

    teleop_arg = DeclareLaunchArgument(
        "teleop",
        default_value="true",
        description="Run the joystick manual override from this machine",
    )
    controller_type_arg = DeclareLaunchArgument(
        "controller_type",
        default_value="taranis",
        # Constrained because controller.launch.py falls back to the 8bitdo config for any
        # type it does not recognise, so a typo would map the wrong axes without an error.
        choices=["taranis", "xbox", "logitech", "8bitdo"],
        description="Controller config for the joystick override",
    )

    # The manual override. Outside processing_nodes so rviz_only:=true cannot drop it, and
    # scoped so its `type`, `debug` etc. stay out of the rest of this file.
    teleop_launch = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("teleop"), "launch", "controller.launch.py"]
                    )
                ),
                launch_arguments={
                    "type": LaunchConfiguration("controller_type"),
                    "debug": "false",
                }.items(),
            )
        ],
        scoped=True,
        condition=IfCondition(teleop),
    )

    # The environment below matches description/launch/view_perseus.launch.py: RViz needs
    # xcb rather than wayland to render reliably here, and the deeper QoS history keeps
    # displays from dropping messages on a busy link.
    rviz_env = {
        "NIXPKGS_ALLOW_UNFREE": "1",
        "QT_QPA_PLATFORM": "xcb",
        "QT_SCREEN_SCALE_FACTORS": "1",
        "RMW_QOS_POLICY_HISTORY": "keep_last",
        "RMW_QOS_POLICY_DEPTH": "100",
    }

    rviz_nixgl = ExecuteProcess(
        condition=IfCondition(use_nixgl),
        cmd=[
            "nix",
            "run",
            "--impure",
            "github:nix-community/nixGL",
            "--",
            "rviz2",
            "-d",
            rviz_config,
        ],
        output="screen",
        additional_env=rviz_env,
    )

    # Plain node rather than ExecuteProcess so use_sim_time can be passed as a parameter;
    # the nixGL path above cannot take one, since it is a bare command line.
    rviz_plain = Node(
        condition=UnlessCondition(use_nixgl),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        additional_env=rviz_env,
    )

    # The rover Draco-encodes the downsampled Livox scan and /Laser_map to save link
    # bandwidth, so what arrives here is not PointCloud2 and RViz cannot subscribe to it.
    # These decoders turn each stream back into a cloud on
    # /livox/lidar/downsampled/decompressed and /Laser_map/downsampled/decompressed.
    #
    # Check base_station.rviz before relying on those topics: its cloud displays have
    # historically pointed at the raw, uncompressed names. The decoders are brought up
    # here so the decompressed clouds exist to be selected either way.
    #
    # Scoped for the same reason the includes in localisation.launch.py are: an include's
    # launch_arguments otherwise land in the enclosing context and are inherited by
    # whatever is included after it.
    decompress_launch = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("sensors"),
                            "launch",
                            "point_cloud_decompress.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                }.items(),
            )
        ],
        scoped=True,
        condition=IfCondition(decompress),
    )

    # Off by default: the intended base-station display is the RViz panel
    # (rviz_plugins/ArenaMinimapPanel), which draws the same layout as a 2D
    # overhead minimap with a heading arrow, and needs no topic at all beyond
    # the pose. This node publishes the same zones as 3D MarkerArrays into the
    # main view instead, which is useful only if you want them alongside the
    # mesh rather than in their own panel.
    minimap_arg = DeclareLaunchArgument(
        "minimap",
        default_value="false",
        description=(
            "Publish the arena zones as 3D markers in the main view. The RViz "
            "ArenaMinimapPanel is the usual display and needs neither this node "
            "nor its topics."
        ),
    )
    minimap = LaunchConfiguration("minimap")

    mesh_arg = DeclareLaunchArgument(
        "mesh",
        default_value="true",
        description="Reconstruct a surface mesh from the decoded map cloud and publish it "
        "as a TRIANGLE_LIST marker on /mesh. Needs decompress:=true.",
    )

    # Reconstructs a surface from the decoded map cloud, so the map can be looked at as
    # terrain rather than as points.
    #
    # It runs here rather than on the rover on purpose. A mesh is far larger on the wire
    # than the cloud it is built from -- a TRIANGLE_LIST marker carries three float64
    # vertices per triangle with no index buffer, roughly ten times the bytes -- so building
    # it at this end means the rover keeps sending the compressed cloud it already sends and
    # the mesh never touches the radio link.
    #
    # Greedy projection triangulation, which connects neighbouring points and leaves gaps as
    # holes. That suits a LiDAR map: it is a thin shell with nothing behind it, and a
    # watertight method would invent ground across every unscanned gap.
    #
    # The input is the decoded map, so this needs decompress:=true.
    mesh_node = Node(
        package="sensors",
        executable="cloud_mesher",
        name="cloud_mesher",
        output="screen",
        parameters=[
            {
                "cloud_topic": "/Laser_map/downsampled/decompressed",
                "mesh_topic": "/mesh",
                # The cloud arriving here has already been voxel downsampled once on the
                # rover, so this mostly bounds the triangulation cost rather than throwing
                # away detail. search_radius has to stay comfortably above it or points end
                # up too far apart to connect and nothing is produced.
                "leaf_size": 0.15,
                "search_radius": 0.6,
                "normal_k": 20,
                "max_surface_angle_deg": 45.0,
                # Normal estimation is the parallel stage, and OpenMP would otherwise take
                # every core on the machine for the fraction of a second it runs. Four is
                # ample: a mesh costs about a core-second of work whatever it is spread
                # across, and at one map every five seconds that is a fifth of a core.
                "num_threads": 4,
                "colour_by_height": True,
                "use_sim_time": use_sim_time,
            }
        ],
        condition=IfCondition(mesh),
    )

    # Draws the arena outline and the rover from a local copy of
    # arena_layout.json, so the base station gets a map without the robot
    # streaming one. The only thing over the link is /arena/robot_pose, which is
    # a PoseStamped at a few Hz -- next to the mesh above, free.
    minimap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("autonomy_bringup"),
                    "launch",
                    "arena_minimap.launch.py",
                ]
            )
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(minimap),
    )

    # rviz_only:=true drops this whole group, regardless of decompress/mesh/minimap,
    # leaving just the RViz processes above.
    processing_nodes = GroupAction(
        [
            decompress_launch,
            mesh_node,
            minimap_launch,
        ],
        condition=UnlessCondition(rviz_only),
    )

    return LaunchDescription(
        [
            rviz_config_arg,
            minimap_arg,
            use_sim_time_arg,
            use_nixgl_arg,
            decompress_arg,
            mesh_arg,
            rviz_only_arg,
            teleop_arg,
            controller_type_arg,
            rviz_nixgl,
            rviz_plain,
            processing_nodes,
            teleop_launch,
        ]
    )
