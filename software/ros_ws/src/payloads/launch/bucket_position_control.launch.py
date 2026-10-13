"""Named-position control of the excavation bucket.

Starts bucket_position_control, which turns a position name published on
`bucket_position` into bank-level SET_POSITION frames. Positions are defined in
config/bucket_positions.yaml.

Do not run alongside bucket_teleop.launch.py or bucket.launch.py: the firmware
picks its control mode from whichever command it received last, so two senders
fight over the banks.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ARGUMENTS
    can_bus = LaunchConfiguration("can_bus", default="can0")

    arguments = [
        DeclareLaunchArgument(
            "can_bus",
            default_value="can0",
            description="CAN interface the bucket controller is on",
        ),
    ]

    positions_config = os.path.join(
        get_package_share_directory("payloads"), "config", "bucket_positions.yaml"
    )

    # NODES
    bucket_position_control = Node(
        package="payloads",
        executable="bucket_position_control",
        output="both",
        parameters=[positions_config, {"can_bus": can_bus}],
    )

    nodes = [bucket_position_control]

    return LaunchDescription(arguments + nodes)
