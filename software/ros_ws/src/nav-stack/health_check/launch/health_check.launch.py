"""Launch the health monitor with the default watch list.

The monitor publishes a single interfaces/msg/SystemHealth snapshot per cycle on
`/health_check/health`, covering the rate, bandwidth and staleness of every topic in the
watch list.

Arguments:
    config_file  path to the parameter file, to point the monitor at a different
                 watch list without editing the installed default
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("health_check"), "config", "health_check.yaml"]
        ),
        description="Parameter file holding the watch list",
    )

    health_monitor = Node(
        package="health_check",
        executable="health_monitor",
        name="health_monitor",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
    )

    return LaunchDescription([config_file_arg, health_monitor])
