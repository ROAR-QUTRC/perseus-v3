"""Launch the stereo visual odometry node with the shared vision configuration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description for the stereo_odometry node."""
    vision_dir = get_package_share_directory("vision")
    config_file = os.path.join(vision_dir, "config", "vision.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulated time"
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            Node(
                package="vision",
                executable="stereo_odometry",
                name="stereo_odometry",
                parameters=[
                    config_file,
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                output="screen",
            ),
        ]
    )
