"""Launch the link monitor against the rover devices in config/network_devices.toml.

Standalone:
  ros2 launch link_monitor link_monitor.launch.py
  ros2 launch link_monitor link_monitor.launch.py interface:=wlp3s0

Publishes /link_health: throughput of the interface the rover is reached over, and the
ping round trip and loss to every enabled "*-brain" device in the config. Meant for the
base station, which sees all the rover's traffic cross the one radio link. It is
included by autonomy_bringup's base_station.launch.py.

Arguments:
    devices_config  path to a network_devices.toml, to monitor a different set of devices
    interface       network interface to read throughput from. Empty detects it from the
                    route to the rover, so it follows a switch between wifi and ethernet
"""

import os
import tomllib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    config_path = LaunchConfiguration("devices_config").perform(context)
    if not config_path:
        config_path = os.path.join(
            get_package_share_directory("link_monitor"),
            "config",
            "network_devices.toml",
        )
    with open(config_path, "rb") as config_file:
        devices = tomllib.load(config_file)

    # The rover's computers. Other entries are people's machines, which come and go and
    # are not part of the link being watched.
    names = []
    ips = []
    for name, device in devices.items():
        if name.endswith("-brain") and device.get("enabled", True):
            names.append(name)
            ips.append(device["ip"])

    return [
        Node(
            package="link_monitor",
            executable="link_monitor",
            name="link_monitor",
            output="screen",
            parameters=[
                {
                    "device_names": names,
                    "device_ips": ips,
                    "interface": LaunchConfiguration("interface").perform(context),
                    "use_sim_time": False,
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "devices_config",
                default_value="",
                description="network_devices.toml to read the devices from; empty uses "
                "the copy installed with this package",
            ),
            DeclareLaunchArgument(
                "interface",
                default_value="",
                description="Interface to read throughput from; empty detects it",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
