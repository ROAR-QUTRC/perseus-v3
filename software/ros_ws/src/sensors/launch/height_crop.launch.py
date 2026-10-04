"""Crop the ceiling out of the Livox scan and the BIEVR-LIO map.

Runs one sensors/height_crop per stream (config/height_crop.yaml):

    /livox/lidar -> /livox/lidar/cropped   for the traversability nodes and downlink
    /Laser_map   -> /Laser_map/cropped     for the downlink

BIEVR-LIO keeps reading the raw /livox/lidar. Needs odom <- livox_frame from TF,
so it runs on the rover with localisation.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

STREAMS = ("livox", "laser_map")


def generate_launch_description():
    """Build the launch description for both height crop nodes."""
    config_file = os.path.join(
        get_package_share_directory("sensors"), "config", "height_crop.yaml"
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulated time"
    )
    use_sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}

    nodes = [
        Node(
            package="sensors",
            executable="height_crop",
            name=f"{prefix}_height_crop",
            parameters=[config_file, use_sim_time],
            output="screen",
        )
        for prefix in STREAMS
    ]
    return LaunchDescription([use_sim_time_arg, *nodes])
