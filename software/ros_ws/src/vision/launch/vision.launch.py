"""Launch the whole vision pipeline in one component container.

Both detectors and the overlay run as composable nodes inside a single process with
intra-process comms enabled, so the camera image is handed between them by pointer
instead of being copied and serialised once per subscriber.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    """Build the composed vision pipeline, including only the enabled nodes."""
    vision_dir = get_package_share_directory("vision")
    config_dir = os.path.join(vision_dir, "config")
    config_file = os.path.join(config_dir, "vision.yaml")

    common_parameters = [
        config_file,
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
    ]

    # extra_arguments enables intra-process comms, which is what makes composing
    # these nodes cheaper than running them as separate processes.
    intra_process = [{"use_intra_process_comms": True}]

    composable_nodes = []

    if LaunchConfiguration("aruco").perform(context) == "true":
        composable_nodes.append(
            ComposableNode(
                package="vision",
                plugin="vision::ArucoDetector",
                name="aruco_detector",
                parameters=common_parameters,
                extra_arguments=intra_process,
            )
        )

    if LaunchConfiguration("cube").perform(context) == "true":
        composable_nodes.append(
            ComposableNode(
                package="vision",
                plugin="vision::CubeDetector",
                name="cube_detector",
                parameters=common_parameters,
                extra_arguments=intra_process,
            )
        )

    if LaunchConfiguration("overlay").perform(context) == "true":
        composable_nodes.append(
            ComposableNode(
                package="vision",
                plugin="vision::DetectionOverlay",
                name="detection_overlay",
                parameters=common_parameters,
                extra_arguments=intra_process,
            )
        )

    if not composable_nodes:
        return []

    vision_container = ComposableNodeContainer(
        name="vision_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=composable_nodes,
        output="screen",
    )

    return [vision_container]


def generate_launch_description():
    """Build the launch description for the composed vision pipeline."""
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulated time"
    )
    cube_arg = DeclareLaunchArgument(
        "cube", default_value="false", description="Enable the cube detector"
    )
    aruco_arg = DeclareLaunchArgument(
        "aruco", default_value="true", description="Enable the ArUco detector"
    )
    overlay_arg = DeclareLaunchArgument(
        "overlay", default_value="true", description="Enable the detection overlay"
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            cube_arg,
            aruco_arg,
            overlay_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
