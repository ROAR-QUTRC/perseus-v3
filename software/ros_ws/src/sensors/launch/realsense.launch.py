"""
RealSense camera launch.

Every RealSense driver setting lives in `config/realsense.yaml`. This launch file
deliberately exposes nothing but the stream toggles, since those are the only settings
worth changing per run:

    enable_depth       depth stream
    enable_infra_pair  the infra1/infra2 IR pair (infra0 is a config file setting)
    enable_color       color stream

The defaults match the baseline setup: color and depth on, IR off.

The driver runs as a *composable* node inside a container rather than as a standalone
process, which is the whole point of this file's shape. At 848x480 the infra pair is
407 KB per frame, 24 MB/s for the pair at 30 fps, and every byte of that was being
serialised through CycloneDDS and multicast onto the wire -- measured at 12.3 MB/s
egressing the LiDAR NIC, with 86% of frames dropped between the sensor and the topic
(the hardware frame counter showed a clean 29.8 fps going in, 4.2 Hz coming out).
A consumer loaded into this same container talks to the driver through intra-process
comms instead, so the frames never reach the middleware at all. autonomy_bringup's
localisation.launch.py loads vision's stereo_odometry here for exactly that reason, under
its `enable_sensors` argument -- stereo odometry is a pose source, so it is owned by the
localisation stack rather than by the sensor pack, and only its *placement* belongs here.

Note the container is `component_container_mt`: the driver runs a callback per stream,
and the single-threaded container would serialize infra1 against infra2 and reintroduce
the stall this arrangement exists to remove.

This replaces an include of realsense2_camera's own rs_launch.py, which builds a plain
Node and offers no way to compose. The parameter handling below mirrors what that file
does -- read the YAML into a dict, hand it to the node, let the stream toggles override
-- with one deliberate omission: rs_launch.py also passes its own launch-argument
defaults for every setting the YAML does not mention, and those are dropped here in
favour of the driver's own defaults. config/realsense.yaml is comprehensive enough that
the two agree, but that is the reason to change this file rather than the YAML if a
setting ever appears to be ignored.

rs_launch.py also picks between Node and LifecycleNode from its global_settings.yaml.
That file ships `use_lifecycle_node: false`, so the plain node is what was running
before and a composable node is a faithful substitution. If a future realsense2_camera
flips that default, this file has to grow the same branch.

Compressed topics need no nodes here. The driver publishes through image_transport, so
every image topic gains a `/compressed` (and depth a `/compressedDepth`) companion from
the plugins sensors depends on.

Usage:
    ros2 launch sensors realsense.launch.py

    ros2 launch sensors realsense.launch.py \
        enable_infra_pair:=true enable_color:=false
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

CONFIG_FILE_NAME = "realsense.yaml"

# The container other launch files load consumers into. autonomy_bringup's
# localisation.launch.py names this same string when it loads stereo_odometry, so the two
# have to agree; it is declared as a launch argument below rather than hardcoded at the use
# site, so a second camera can be brought up under a different container name.
DEFAULT_CONTAINER_NAME = "sensor_container"


def generate_launch_description():
    """Build the launch description for the RealSense driver."""
    config_file = str(
        Path(get_package_share_directory("sensors")) / "config" / CONFIG_FILE_NAME
    )
    with open(config_file, "r") as f:
        driver_params = yaml.safe_load(f)

    # camera_name and camera_namespace are the node's identity rather than settings it
    # reads, so they have to be resolved here in Python: ComposableNode takes them as
    # construction arguments, where rs_launch.py could leave them as substitutions.
    # Everything downstream is named for the result -- /camera/camera/... -- including
    # vision.yaml's topic defaults and ekf_config.yaml's odom1.
    camera_name = driver_params.get("camera_name", "camera")
    camera_namespace = driver_params.get("camera_namespace", "camera")

    enable_depth_arg = DeclareLaunchArgument(
        "enable_depth", default_value="false", description="Enable the depth stream"
    )
    enable_infra_pair_arg = DeclareLaunchArgument(
        "enable_infra_pair",
        default_value="true",
        description="Enable the infra1/infra2 IR stream pair",
    )
    enable_color_arg = DeclareLaunchArgument(
        "enable_color", default_value="true", description="Enable the color stream"
    )
    container_name_arg = DeclareLaunchArgument(
        "container_name",
        default_value=DEFAULT_CONTAINER_NAME,
        description="Name of the component container the driver runs in. Consumers that "
        "want intra-process access to the image streams load themselves into this same "
        "container -- see localisation.launch.py's enable_sensors argument.",
    )

    # The stream toggles are kept out of the YAML and supplied here, and the ordering
    # below is what makes that work: later entries win, so these override anything the
    # config file happens to say. rs_launch.py had the opposite precedence, which is why
    # the toggles could not live in the YAML in the first place.
    realsense_container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="realsense2_camera",
                plugin="realsense2_camera::RealSenseNodeFactory",
                name=camera_name,
                namespace=camera_namespace,
                parameters=[
                    driver_params,
                    {
                        "enable_depth": LaunchConfiguration("enable_depth"),
                        "enable_color": LaunchConfiguration("enable_color"),
                        # infra1 and infra2 are the left/right halves of one stereo
                        # pair; the driver takes them as separate flags but they are
                        # only useful together.
                        "enable_infra1": LaunchConfiguration("enable_infra_pair"),
                        "enable_infra2": LaunchConfiguration("enable_infra_pair"),
                    },
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        ],
    )

    return LaunchDescription(
        [
            enable_depth_arg,
            enable_infra_pair_arg,
            enable_color_arg,
            container_name_arg,
            realsense_container,
        ]
    )
