"""Launch the system monitor, namespaced by the name of the machine it runs on.

Standalone:
  ros2 launch system_monitor system_monitor.launch.py
  ros2 launch system_monitor system_monitor.launch.py device_name:=jetson

Also included by localisation.launch.py and perseus.launch.py, so every computer
running part of the stack publishes /<device_name>/system_health and shows up as its own
row in the base station's Machine Health panel.
"""

import re
import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def sanitize_ros_name(name):
    """Make a hostname legal as a ROS namespace: [A-Za-z0-9_], not starting with a digit."""
    sanitized = re.sub(r"[^A-Za-z0-9_]", "_", name)
    if not sanitized or sanitized[0].isdigit():
        sanitized = "d_" + sanitized
    return sanitized


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "device_name",
                default_value=sanitize_ros_name(socket.gethostname()),
                description="Name of this machine. Used as the namespace, so the "
                "topic is /<device_name>/system_health. Must be unique per machine.",
            ),
            DeclareLaunchArgument(
                "publish_rate_hz",
                default_value="1.0",
                description="How often to publish, in Hz.",
            ),
            Node(
                package="system_monitor",
                executable="system_monitor",
                name="system_monitor",
                namespace=LaunchConfiguration("device_name"),
                output="screen",
                parameters=[
                    {
                        "device_name": LaunchConfiguration("device_name"),
                        "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                    }
                ],
            ),
        ]
    )
