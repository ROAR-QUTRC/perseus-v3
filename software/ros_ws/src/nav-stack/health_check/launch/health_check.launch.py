"""Launch the health monitor with the default watch list.

The monitor publishes a single interfaces/msg/SystemHealth snapshot per cycle on
`/health_check/health`, covering topic rates, per-topic bandwidth, and network link
counters.

Arguments:
    config_file     path to the parameter file, to point the monitor at a different
                    watch list without editing the installed default
    ping_host       host to probe for reachability; overrides the config file, and
                    defaults to empty there so probing is off unless asked for
    interface_name  interface the link counters are read from; overrides the config
                    file, so a robot moving between the wireless and a tethered link
                    does not need a rebuilt config

Both overrides are applied after the parameter file, so setting either here wins over
whatever the file says.
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
    ping_host_arg = DeclareLaunchArgument(
        "ping_host",
        default_value="",
        description="Host to probe for reachability; empty disables probing",
    )
    interface_name_arg = DeclareLaunchArgument(
        "interface_name",
        default_value="wlan0",
        description="Interface the throughput and error counters are read from. An "
        "interface that exists but has no carrier reports present with every counter at "
        "zero, so this has to name the link actually carrying traffic.",
    )

    health_monitor = Node(
        package="health_check",
        executable="health_monitor",
        name="health_monitor",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "ping_host": LaunchConfiguration("ping_host"),
                "interface_name": LaunchConfiguration("interface_name"),
            },
        ],
    )

    return LaunchDescription(
        [config_file_arg, ping_host_arg, interface_name_arg, health_monitor]
    )
