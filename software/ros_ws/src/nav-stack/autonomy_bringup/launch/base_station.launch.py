"""Base station view: RViz and the point cloud decoders, for an operator machine
watching a robot elsewhere.

Nothing here drives the robot. This launches the visualisation side alone, so it can be
run on a laptop that shares a ROS domain with the rover while the sensing, estimation and
health monitoring all run on the rover itself.

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
                  for RViz. Off by default; see the comment on the node below for the cost
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
    # Note that base_station.rviz does not yet display those topics: its Livox Cloud and
    # Cloud Map displays are still pointed at /cloud_registered and /Laser_map, the raw
    # names. The decoders are brought up here so the decompressed clouds exist to be
    # selected; retargeting the config is a separate change.
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

    mesh_arg = DeclareLaunchArgument(
        "mesh",
        default_value="false",
        description="Reconstruct a surface mesh from the decoded map cloud and publish it "
        "as a TRIANGLE_LIST marker for RViz. Needs decompress:=true. NOT YET WORKING -- see "
        "the comment on the node below.",
    )

    # ImMesh's incremental mesher, without the LiDAR odometry it normally comes with -- the
    # rover already does that, and the mesher only ever needed a world-frame cloud plus the
    # pose it was seen from.
    #
    # It runs here rather than on the rover on purpose. A meshed surface is roughly an order
    # of magnitude larger on the wire than the cloud it is built from, because a
    # TRIANGLE_LIST carries three float64 vertices per triangle with no index buffer. Built
    # at the base station it never crosses the radio link at all: the rover sends the same
    # compressed cloud it already sends, and the marker goes straight to RViz on this
    # machine.
    #
    # The input is the decoded map, so this only does anything with decompress:=true. The
    # node forwards only points it has not already meshed -- the map arrives whole every
    # time -- and takes the viewpoint from /Odometry, which it needs because an accumulated
    # map has no vantage point of its own and the mesher uses one to orient each triangle.
    #
    # NOT YET WORKING, and off by default for that reason. It builds, subscribes, and the
    # incremental filtering does its job -- 17k new points on the first map, a few hundred
    # after -- but ImMesh's meshing thread then dies in Triangle_manager::get_triangle_center,
    # dereferencing a point id its triangle still refers to and the point store no longer
    # holds. That is upstream code, reached by a usage it was not written for: it assumes a
    # monotonically growing store fed one scan at a time from a moving viewpoint, and an
    # accumulated map re-fed whole every few seconds is neither. Feeding the per-scan cloud
    # (/livox/lidar/downsampled/decompressed plus the odometry pose) is the shape it expects
    # and the likelier fix; guarding the id lookups inside ImMesh is the other.
    mesh_node = Node(
        package="immesh_ros2",
        executable="immesh_node",
        name="immesh",
        output="screen",
        parameters=[
            {
                "cloud_topic": "/Laser_map/downsampled/decompressed",
                "odom_topic": "/Odometry",
                "mesh_topic": "/mesh",
                "map_frame": "odom",
                # Coarser than the rover's map resolution: the cloud arriving here has
                # already been voxel downsampled once before Draco, so asking for finer
                # detail than that only costs triangles without adding surface.
                "meshing_voxel_size": 0.4,
                "minimum_point_distance": 0.15,
                "new_point_resolution": 0.15,
                "min_new_points": 100,
                "max_threads": 4,
                "mesh_publish_interval_s": 5.0,
                "ply_path": "",
                "use_sim_time": use_sim_time,
            }
        ],
        condition=IfCondition(mesh),
    )

    return LaunchDescription(
        [
            rviz_config_arg,
            use_sim_time_arg,
            use_nixgl_arg,
            decompress_arg,
            mesh_arg,
            rviz_nixgl,
            rviz_plain,
            decompress_launch,
            mesh_node,
        ]
    )
