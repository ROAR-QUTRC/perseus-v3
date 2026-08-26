from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("sensors"),
        "config",
        "dead_zone_filter.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="sensors",
                executable="filter_node",
                name="lidar_dead_zone_filter",
                output="screen",
                parameters=[config],
            ),
        ]
    )
