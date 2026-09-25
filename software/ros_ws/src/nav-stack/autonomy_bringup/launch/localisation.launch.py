#!/usr/bin/env python3
"""Launch the localisation stack: LiDAR-inertial odometry fused by the EKF.

Three pieces, and the split matters:

  1. BIEVR-LIO, the LiDAR-inertial odometry, configured by config/bievr_mid360.yaml. It
     reads the raw Livox cloud off /livox/lidar, publishes /Odometry as odom -> base_link
     (remapped from its own /bievr_lio/odom), and broadcasts no TF of its own.
  2. ekf.launch.py runs robot_localization from config/ekf_config.yaml, fusing that pose
     with IMU angular velocity and owning the odom -> base_link transform.
Two monitors ride along with them, both watching what this stack produces rather than
adding to it: the mobility watchdog, which compares commanded velocity against the EKF
output to catch sustained wheel slip, and the health monitor, which reports the rate and
staleness of the topics above on /health_check/health.

Start order does not matter. BIEVR-LIO withholds /Odometry until the LiDAR frame ->
base_link lookup resolves in TF, so until robot_state_publisher is up the EKF runs on the
IMU alone.

BIEVR-LIO also publishes its map on /Laser_map and saves it on /map_save. It keeps one
height image per voxel and reprojects it on request, so the map is deduplicated, capped by
map.max_size, and costs a walk over the whole map each time it is published. Hence
publish.map_interval_s in bievr_mid360.yaml: seconds between publishes, not per scan.

The config is tuned for the real robot. sim:=true layers config/bievr_mid360_sim.yaml on
top for Gazebo, which publishes a differently shaped cloud: BIEVR-LIO merges its two config
files per key, so the sim's differences are just the handful of keys that file sets.

A second, independent pose source is fused alongside BIEVR-LIO: vision's
stereo_odometry (libviso2 against the RealSense infra1/infra2 pair), which arrives as odom1
in ekf_config.yaml. How it is brought up depends on `enable_sensors:=`, and the difference
is not cosmetic:

  enable_sensors:=false (default)  The sensor drivers are assumed to be running already,
                                   from sensors/sensors.launch.py in their own terminal.
                                   stereo_odometry is launched here as a standalone node.
  enable_sensors:=true             This file brings up the Livox and RealSense drivers
                                   itself, and loads stereo_odometry as a *component* into
                                   the container realsense.launch.py creates.

The composed path exists because the infra pair is 848x480 Y8 at 30 fps -- 407 KB a frame,
24 MB/s for the two together. Run as a separate process every byte of that is serialised
through the middleware, and measured on this rover 86% of frames never arrived: 29.8 fps at
the camera's own frame counter against 4.2 Hz on the topic, with infra1 and infra2 drifting
to different rates so libviso2 was matching frames that did not correspond to each other.
Loaded into the driver's container the frames are passed by pointer instead, and both
streams measure a clean 30 Hz. So prefer enable_sensors:=true whenever this file is the
thing starting the camera.

The default is false because nothing in this repo orchestrates bringup -- the drivers are
started by hand -- and defaulting to true would open the RealSense a second time alongside
an existing one and fail with "Device or resource busy".

Either way the EKF only names the topic and fuses whatever appears on it, so nothing
downstream branches on this choice.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


# Must match realsense.launch.py's DEFAULT_CONTAINER_NAME. That file creates the container,
# this one loads stereo_odometry into it by name, and a mismatch does not raise:
# LoadComposableNodes simply waits for a container that never appears.
SENSOR_CONTAINER_NAME = "sensor_container"


def bievr_actions(rviz, use_sim_time, is_sim, params_file):
    """BIEVR-LIO: the node directly, rather than its own launch file.

    BIEVR-LIO takes two config paths and merges them per leaf key, so the simulator's
    differences go in a second file that wins on the handful of keys it sets -- see
    bievr_mid360_sim.yaml. bievr_lio_ros2's own launch file takes the same two paths but
    resolves them inside its own share directory, and hardcodes its RViz config, which is
    the only reason not to include it.

    Neither file is a ROS parameter file -- the node parses them itself with yaml-cpp -- so
    they go on the command line. use_sim_time is a genuine ROS parameter and stays one.

    BIEVR-LIO namespaces everything it publishes under /bievr_lio; the remaps put its
    odometry and map where ekf_config.yaml's odom0 and the point cloud link expect them.
    """

    def config(name):
        return os.path.join(
            get_package_share_directory("autonomy_bringup"), "config", name
        )

    # params_file replaces the main config outright rather than layering on it: the second
    # slot is already the sim's, and the node takes only two.
    config_args = ["--params_file", params_file or config("bievr_mid360.yaml")]
    if is_sim:
        config_args += ["--sensor_config_file", config("bievr_mid360_sim.yaml")]

    return [
        Node(
            package="bievr_lio_ros2",
            executable="process_topics",
            name="bievr_lio",
            output="screen",
            arguments=config_args,
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[
                ("/bievr_lio/odom", "/Odometry"),
                # /Laser_map is what the Draco compressor below and the base station
                # subscribe to.
                ("/bievr_lio/map", "/Laser_map"),
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [FindPackageShare("bievr_lio_ros2"), "rviz", "config.rviz"]
                ),
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(rviz),
        ),
    ]


def launch_setup(context, *args, **kwargs):
    """Build the actions once the launch arguments can be resolved.

    An OpaqueFunction is needed because `sim`, `enable_sensors` and `bievr_params_file` are
    needed as real Python values here rather than as substitutions: they decide which
    actions exist at all and what goes on BIEVR-LIO's command line.
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    ekf_params_file = LaunchConfiguration("ekf_params_file")
    sim = LaunchConfiguration("sim")
    is_sim = sim.perform(context).lower() == "true"
    enable_sensors = (
        LaunchConfiguration("enable_sensors").perform(context).lower() == "true"
    )

    lio_actions = bievr_actions(
        rviz,
        use_sim_time,
        is_sim,
        LaunchConfiguration("bievr_params_file").perform(context),
    )

    # Reused rather than duplicated, so the EKF node and its parameters are defined once.
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("autonomy_bringup"), "launch", "ekf.launch.py"]
            )
        ),
        launch_arguments={
            "params_file": ekf_params_file,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # The second pose source, plus the sensors it feeds on. Which of these two branches is
    # built is the whole of what `enable_sensors` selects -- the module docstring has the
    # measurements that make the composed one worth reaching for.
    #
    # Resolved to a Python bool rather than driven by IfCondition because the branches are
    # mutually exclusive and one of them is better off never being *constructed*:
    # livox.launch.py reads the host address with `ip -4 addr show <interface>` inside its
    # own OpaqueFunction and raises RuntimeError when that interface has no IPv4. Under a
    # false IfCondition that body would not run either, but not building it at all keeps the
    # default path from carrying an interface name it never uses.
    if enable_sensors:
        sensor_actions = [
            GroupAction(
                [
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("sensors"),
                                    "launch",
                                    "livox.launch.py",
                                ]
                            )
                        ),
                        launch_arguments={
                            "interface": LaunchConfiguration("interface"),
                        }.items(),
                    )
                ],
                scoped=True,
            ),
            # Creates the component container the load below targets. No use_sim_time to
            # pass: the driver timestamps from the camera's own clock regardless, and
            # realsense.launch.py declares no such argument to receive it.
            GroupAction(
                [
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("sensors"),
                                    "launch",
                                    "realsense.launch.py",
                                ]
                            )
                        ),
                        # Pinned rather than left to realsense.launch.py's identical
                        # default, and this is not belt-and-braces. GroupAction(scoped=True)
                        # forwards the enclosing configurations inward, and `ros2 launch`
                        # accepts any k:=v the file never declared, so a stray
                        # `container_name:=foo` would reach this include, rename the
                        # container, and leave the load below waiting on a service that
                        # never appears -- LoadComposableNodes waits without a timeout and
                        # logs nothing above DEBUG, so the symptom is a launch that hangs
                        # in silence. An explicit argument wins inside the scope.
                        launch_arguments={
                            "container_name": SENSOR_CONTAINER_NAME,
                        }.items(),
                    )
                ],
                scoped=True,
            ),
            # Into that container, which is the entire point of this branch. The name and
            # namespace are not free choices: vision.yaml keys this node's parameters under
            # a top-level `stereo_odometry:` block at the root namespace, so renaming it
            # here would silently drop every one of them and leave the node on its built-in
            # defaults -- including publish_tf, which must stay false while the EKF owns
            # odom -> base_link.
            LoadComposableNodes(
                target_container=SENSOR_CONTAINER_NAME,
                composable_node_descriptions=[
                    ComposableNode(
                        package="vision",
                        plugin="vision::StereoOdometry",
                        name="stereo_odometry",
                        namespace="",
                        parameters=[
                            os.path.join(
                                get_package_share_directory("vision"),
                                "config",
                                "vision.yaml",
                            ),
                            {"use_sim_time": use_sim_time},
                        ],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    )
                ],
            ),
        ]
    else:
        # The same node and the same config file, in its own process. vision's own launch
        # file is reused rather than a Node() rebuilt here so the two paths cannot drift.
        # This is also the branch the simulator wants: Gazebo publishes the infra topics
        # with no RealSense driver present, so there is no container to load into.
        sensor_actions = [
            GroupAction(
                [
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("vision"),
                                    "launch",
                                    "stereo_odometry.launch.py",
                                ]
                            )
                        ),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                        }.items(),
                    )
                ],
                scoped=True,
            )
        ]

    # Flattens the EKF's odom -> base_link into odom -> base_footprint, dropping z, roll
    # and pitch while keeping yaw. nav2's costmaps and controller are the consumers, but
    # the transform is derived from the EKF output, so it is brought up here rather than
    # with navigation: anything reading base_footprint then gets it as soon as
    # localisation is running, without nav2 having to be up.
    flat_footprint_broadcaster_node = Node(
        package="footprint_broadcaster",
        executable="flat_footprint_broadcaster",
        name="flat_footprint_broadcaster",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("autonomy_bringup"),
                    "config",
                    "footprint_broadcaster.yaml",
                ]
            ),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    # Owns the arena frame: seeds map -> odom from its initial_pose parameter so the
    # zone layout is usable straight away, and refines it when /arena/localise is
    # called against the rail fiducials. Brought up with localisation because it
    # completes the frame chain -- the EKF gives odom -> base_link, this gives
    # map -> odom, and nav2 needs a global frame to plan in.
    #
    # It does not localise continuously and nothing here calls the service: the
    # camera is lazy, so between requests it renders nothing and costs nothing.
    # Whatever drives the acquisition behaviour is expected to call it.
    arena_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("autonomy_bringup"),
                    "launch",
                    "arena_server.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Watches /odometry/filtered against /cmd_vel_out for wheel slip, so it belongs
    # wherever the EKF that produces /odometry/filtered is brought up.
    watchdog_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("watchdog"), "launch", "mobility_watchdog.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Measures the rate, bandwidth and staleness of the topics this stack consumes and
    # produces -- the raw and corrected IMU, the Livox cloud, /Odometry, /odom and
    # /odometry/filtered are most of its default watch list -- so it comes up with them.
    # It takes no use_sim_time: rate and staleness are measured against arrival in real
    # time, which is what a stalled publisher shows up in regardless of the clock the
    # rest of the stack is on.
    health_check_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("health_check"), "launch", "health_check.launch.py"]
            )
        )
    )

    # The rover half of the point cloud link: voxel downsamples the live Livox scan and
    # /Laser_map (BIEVR-LIO's map, remapped above and published every
    # publish.map_interval_s), then Draco encodes both for the base station. Its two inputs
    # are exactly what this stack consumes and produces, so it comes up with them; the
    # base station runs sensors/point_cloud_decompress.launch.py against the Draco topics.
    point_cloud_compress_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("sensors"),
                    "launch",
                    "point_cloud_compress.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # Every include is scoped, and it is not optional. IncludeLaunchDescription sets its
    # launch_arguments into the *enclosing* context with no push/pop of its own, and
    # DeclareLaunchArgument only applies a default when the name is not already set. So an
    # argument one include sets is inherited by every include after it, and the inheriting
    # launch file cannot tell that happened.
    #
    # This has bitten before: a since-removed include took an argument called `config_file`,
    # generic enough that health_check takes one by the same name, and unscoped the health
    # monitor was handed the other file's path as its parameter file and came up with an
    # empty watch list -- silently, since a missing watch list is a warning and not an
    # error. Scoping confines each include's arguments to the include that set them.
    return (
        lio_actions
        + sensor_actions
        + [
            GroupAction([ekf_launch], scoped=True),
            flat_footprint_broadcaster_node,
            GroupAction([arena_server_launch], scoped=True),
            GroupAction([watchdog_launch], scoped=True),
            GroupAction([health_check_launch], scoped=True),
            GroupAction([point_cloud_compress_launch], scoped=True),
        ]
    )


def generate_launch_description():
    declare_sim = DeclareLaunchArgument(
        "sim",
        default_value="false",
        description="Adapt the LiDAR parameters to Gazebo's point cloud. You almost always "
        "want use_sim_time:=true alongside it.",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use the simulation clock. Only set true when something publishes "
        "/clock, otherwise the EKF waits at startup and never runs.",
    )

    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Launch RViz with BIEVR-LIO's own display config.",
    )

    declare_ekf_params_file = DeclareLaunchArgument(
        "ekf_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("autonomy_bringup"), "config", "ekf_config.yaml"]
        ),
        description="Parameters file for the robot_localization EKF.",
    )

    declare_bievr_params_file = DeclareLaunchArgument(
        "bievr_params_file",
        default_value="",
        description="Replace config/bievr_mid360.yaml with this file, e.g. to A/B-test "
        "BIEVR-LIO settings against a replayed bag without editing the tracked config. "
        "Empty uses the package's own.",
    )

    declare_enable_sensors = DeclareLaunchArgument(
        "enable_sensors",
        default_value="false",
        # Constrained because launch_setup reads this with a strict `== "true"`, which --
        # unlike IfCondition -- does not accept 1/on/yes, so without choices an
        # `enable_sensors:=1` would read as false and silently skip the sensors.
        choices=["true", "false"],
        description="Bring the Livox and RealSense drivers up from this file, and load "
        "vision's stereo_odometry into the camera's component container -- 30 Hz on the "
        "infra pair, against roughly 4 Hz when it runs as its own process. Leave it false "
        "when the drivers are already running from sensors/sensors.launch.py, in which case "
        "stereo_odometry is launched standalone instead. See the module docstring.",
    )

    declare_interface = DeclareLaunchArgument(
        "interface",
        default_value="eth1",
        description="Network interface the Livox driver reads the host IP from, forwarded "
        "to livox.launch.py and used only when enable_sensors:=true. The default matches "
        "livox.launch.py's own; the driver fails at launch if the interface it names has no "
        "IPv4 address, so this needs to be right per machine.",
    )

    bias_remover_container = ComposableNodeContainer(
        name="imu_bias_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",  # mt is nice for multiple callbacks
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="sensors",
                plugin="imu_processors::BiasEstimator",
                name="imu_bias_estimator",
                parameters=[
                    {
                        "use_odom": True,
                        "use_cmd_vel": False,
                        # EWMA time constant is roughly 1/alpha samples at
                        # estimator_rate_hz: 0.01 at 100 Hz was ~1 s, 0.001 is ~10 s.
                        # A gyro bias drifts over minutes (with temperature), so a
                        # 1 s window mostly measures gyro noise; averaging longer cuts
                        # the estimator's own variance at no real cost in tracking.
                        #
                        # This was changed on the theory that the stationary gate --
                        # which with use_cmd_vel False and mode AND reduces to "wheel
                        # /odom reads below odom_threshold" -- would be fooled while the
                        # wheels slip or stall, letting real rotation land in the bias.
                        # Measured on the rover, that does NOT happen. Peak-to-peak
                        # movement of /livox/gyro_bias z:
                        #
                        #     stationary, 30 s      0.000052 rad/s   mean -0.011472
                        #     driving, 40 s         0.000061 rad/s   mean -0.011453
                        #
                        # i.e. the gate holds and the accumulator stays frozen through a
                        # drive. BIEVR-LIO's own residual estimate on
                        # /bievr_lio/bias/gyro agrees, sitting at -3e-5 rad/s, so the
                        # correction is not fighting the filter's internal one either.
                        #
                        # Both values are therefore fine in practice. 0.001 is kept as
                        # the better-conditioned of the two, not as a fix for anything
                        # observed.
                        "accumulator_alpha": 0.001,
                        "stationary_mode": "AND",  # OR / AND
                        "imu_in_topic": "/livox/imu",
                        "odom_topic": "/odom",  # from the wheel encoder
                        "bias_out_topic": "/livox/gyro_bias",
                        "estimator_rate_hz": 100.0,
                    }
                ],
            ),
            ComposableNode(
                package="sensors",
                plugin="imu_processors::BiasRemover",
                name="imu_bias_remover",
                parameters=[
                    {
                        "imu_in_topic": "/livox/imu",
                        "bias_in_topic": "/livox/gyro_bias",
                        "imu_out_topic": "/livox/imu/corrected",
                        "output_rate_hz": 100.0,
                    }
                ],
            ),
        ],
    )
    return LaunchDescription(
        [
            declare_sim,
            declare_use_sim_time,
            declare_rviz,
            declare_ekf_params_file,
            declare_bievr_params_file,
            declare_enable_sensors,
            declare_interface,
            bias_remover_container,
            OpaqueFunction(function=launch_setup),
        ]
    )
