#!/usr/bin/env python3
"""Launch the localisation stack: LiDAR-inertial odometry fused by the EKF.

Three pieces, and the split matters:

  1. A LiDAR-inertial odometry backend, chosen with `lio:=`. Either one reads the raw Livox
     cloud off /livox/lidar, publishes /Odometry as odom -> base_link, and broadcasts no TF
     of its own:
       bievr (default)     BIEVR-LIO, configured by config/bievr_mid360.yaml, with its
                           /bievr_lio/odom remapped onto /Odometry.
       fast_lio            FAST-LIO 2, configured by config/livox_mid360.yaml.
     Everything downstream sees the same topic and the same frames either way, so the two
     are interchangeable from here on and the rest of this file does not branch on them.
  2. ekf.launch.py runs robot_localization from config/ekf_config.yaml, fusing that pose
     with IMU angular velocity and owning the odom -> base_link transform.
Two monitors ride along with them, both watching what this stack produces rather than
adding to it: the mobility watchdog, which compares commanded velocity against the EKF
output to catch sustained wheel slip, and the health monitor, which reports the rate and
staleness of the topics above on /health_check/health.

Start order does not matter. Both backends withhold /Odometry until the LiDAR frame ->
base_link lookup resolves in TF, so until robot_state_publisher is up the EKF runs on the
IMU alone.

Both backends publish /Laser_map and save a map on /map_save, so the point cloud link and
map saving work either way -- but they build that map differently, and it shows. FAST-LIO
accumulates registered scans, so /Laser_map is every point it ever kept and grows without
bound. BIEVR-LIO keeps one height image per voxel and reprojects it on request, so its map
is deduplicated, capped by map.max_size, and costs a walk over the whole map each time it
is published rather than a growing buffer each scan. Hence publish.map_interval_s in
bievr_mid360.yaml: seconds between publishes, not per scan.

Both configs are tuned for the real robot, and both backends take sim:=true against
Gazebo, which publishes a differently shaped cloud. What changes and why is in
sim_overrides() for fast_lio and in config/bievr_mid360_sim.yaml for bievr. The shape of
the answer differs because the backends do: FAST-LIO takes one parameter file, so its
overrides are applied to a rewritten copy of it, while BIEVR-LIO already layers two configs
and merges them per key, so its overrides are just a second file.
"""

import os
import tempfile

import yaml
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
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def sim_overrides(params):
    """Adjust the real-robot parameters for Gazebo's point cloud, in place.

    Gazebo publishes x,y,z,intensity,ring. The real Livox driver publishes
    reflectivity,tag,line. lidar_type selects which of those layouts FAST-LIO deserialises
    into, so against the sim, type 4 finds no `line` field and reads the scan-line index as 0
    for every point -- collapsing all points onto one line and wrecking the per-point
    timestamps that motion compensation depends on. Type 2 reads `ring`, which the sim does
    provide, and derives times from azimuth when no per-point time is present.

    scan_line is the number of lines to expect: 32 vertical samples in the sim, against 4 real
    scan lines on a MID-360. The Velodyne handler skips points whose ring exceeds it.

    lid_topic needs no override: Gazebo's ±Inf no-return rays are rejected by the preprocess
    handlers now, so the raw topic is safe to consume directly.
    """
    params["preprocess"]["lidar_type"] = 2
    params["preprocess"]["scan_line"] = 32


LIO_BACKENDS = ("fast_lio", "bievr")


def fast_lio_actions(rviz, use_sim_time, is_sim):
    """The FAST-LIO backend: its own launch file, over a rewritten copy of its config.

    The copy is why this cannot be a plain substitution. fast_lio takes a config *path*
    rather than a parameter dictionary, so ~ in map_file_path and the sim overrides both
    have to be applied to a real file before the node starts.
    """
    fast_lio_params_file = os.path.join(
        get_package_share_directory("autonomy_bringup"), "config", "livox_mid360.yaml"
    )
    with open(fast_lio_params_file, "r") as f:
        fast_lio_params = yaml.safe_load(f)

    params = fast_lio_params.get("/**", {}).get("ros__parameters", {})

    if "map_file_path" in params:
        resolved_path = os.path.expanduser(params["map_file_path"])
        params["map_file_path"] = resolved_path
        os.makedirs(os.path.dirname(resolved_path), exist_ok=True)

    if is_sim:
        sim_overrides(params)

    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    yaml.dump(fast_lio_params, tmp)
    tmp.close()

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("fast_lio"), "launch", "mapping.launch.py"]
            )
        ),
        launch_arguments={
            "config_path": os.path.dirname(tmp.name),
            "config_file": os.path.basename(tmp.name),
            "rviz": rviz,
            "use_sim_time": use_sim_time,
        }.items(),
    )
    # Scoped for the same reason every other include here is: see the comment at the end of
    # launch_setup.
    return [GroupAction([fast_lio_launch], scoped=True)]


def bievr_actions(rviz, use_sim_time, is_sim):
    """The BIEVR-LIO backend: the node directly, rather than its own launch file.

    There is no temp-file dance here, and no sim_overrides() either. BIEVR-LIO takes two
    config paths and merges them per leaf key, so the simulator's differences go in a
    second file that wins on the handful of keys it sets -- see bievr_mid360_sim.yaml.
    That is also the only reason not to include bievr_lio_ros2's own launch file: it takes
    the same two paths but resolves them inside its own share directory, and hardcodes its
    RViz config.

    Neither file is a ROS parameter file -- the node parses them itself with yaml-cpp -- so
    they go on the command line. use_sim_time is a genuine ROS parameter and stays one.

    The remap is what makes the two backends interchangeable: BIEVR-LIO namespaces
    everything it publishes under /bievr_lio, and ekf_config.yaml's odom0 wants /Odometry.
    """

    def config(name):
        return os.path.join(
            get_package_share_directory("autonomy_bringup"), "config", name
        )

    config_args = ["--params_file", config("bievr_mid360.yaml")]
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
                # Same reasoning as the odometry remap: /Laser_map is what the Draco
                # compressor below and the base station already subscribe to, so the map
                # arrives under the name FAST-LIO would have published it under.
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

    An OpaqueFunction is needed because both backends need their arguments as real Python
    values here rather than as substitutions: `lio` selects which set of actions exists at
    all, and FAST-LIO's config has to be read and rewritten to a file before its node
    starts.
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    ekf_params_file = LaunchConfiguration("ekf_params_file")
    is_sim = LaunchConfiguration("sim").perform(context).lower() == "true"
    lio = LaunchConfiguration("lio").perform(context).lower()

    # An unknown value never reaches here: DeclareLaunchArgument takes LIO_BACKENDS as its
    # choices and rejects anything else before this function runs.
    lio_actions = (
        fast_lio_actions(rviz, use_sim_time, is_sim)
        if lio == "fast_lio"
        else bievr_actions(rviz, use_sim_time, is_sim)
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
    # FAST-LIO's /Laser_map, then Draco encodes both for the base station. Its two inputs
    # are exactly what this stack consumes and produces, so it comes up with them; the
    # base station runs sensors/point_cloud_decompress.launch.py against the Draco topics.
    # Both backends feed the /Laser_map half: FAST-LIO publishes it directly, BIEVR-LIO
    # publishes /bievr_lio/map on publish.map_interval_s and is remapped onto it above.
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
    # fast_lio makes that concrete: it takes an argument called `config_file`, which is
    # generic enough that health_check takes one by the same name. Unscoped, the health
    # monitor was handed fast_lio's rewritten temp filename as its parameter file and came
    # up with an empty watch list -- silently, since a missing watch list is a warning and
    # not an error. Scoping confines each include's arguments to the include that set them.
    return lio_actions + [
        GroupAction([ekf_launch], scoped=True),
        flat_footprint_broadcaster_node,
        GroupAction([arena_server_launch], scoped=True),
        GroupAction([watchdog_launch], scoped=True),
        GroupAction([health_check_launch], scoped=True),
        GroupAction([point_cloud_compress_launch], scoped=True),
    ]


def generate_launch_description():
    declare_lio = DeclareLaunchArgument(
        "lio",
        default_value="bievr",
        choices=list(LIO_BACKENDS),
        description="Which LiDAR-inertial odometry backend produces /Odometry. "
        "bievr is BIEVR-LIO with config/bievr_mid360.yaml; fast_lio is FAST-LIO 2 with "
        "config/livox_mid360.yaml. Everything downstream is identical either way.",
    )

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
        description="Launch RViz with the selected backend's own display config.",
    )

    declare_ekf_params_file = DeclareLaunchArgument(
        "ekf_params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("autonomy_bringup"), "config", "ekf_config.yaml"]
        ),
        description="Parameters file for the robot_localization EKF.",
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
                        "accumulator_alpha": 0.01,
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
            declare_lio,
            declare_sim,
            declare_use_sim_time,
            declare_rviz,
            declare_ekf_params_file,
            bias_remover_container,
            OpaqueFunction(function=launch_setup),
        ]
    )
