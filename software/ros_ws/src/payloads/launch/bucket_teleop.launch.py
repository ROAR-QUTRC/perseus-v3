"""Manual (teleop) control of the excavation bucket.

Starts the open-loop bucket_driver, which turns the Actuators messages published by
teleop's generic_controller into bank-level SET_SPEED frames.

It runs alongside the ros2_control stack as an operator override. The firmware picks
its control mode from whichever command it received last, so bucket_driver does not
just transmit over a running trajectory: the first stick input past its deadband
deactivates the bucket's command controllers, and only then do speeds go out. The
/bucket/rearm service hands control back to autonomy.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ARGUMENTS
    can_bus = LaunchConfiguration("can_bus", default="can0")
    # Velocity which maps to full duty cycle. Start low on the bench and work
    # up - see the teleop controller configs, which scale sticks to +/-0.1.
    max_actuator_speed = LaunchConfiguration("max_actuator_speed", default="0.1")

    arguments = [
        DeclareLaunchArgument(
            "can_bus",
            default_value="can0",
            description="CAN interface the bucket controller is on",
        ),
        DeclareLaunchArgument(
            "max_actuator_speed",
            default_value="0.1",
            description="Actuator velocity which maps to full duty cycle",
        ),
    ]

    # NODES
    bucket_driver = Node(
        package="payloads",
        executable="bucket_driver",
        output="both",
        parameters=[
            {
                "can_bus": can_bus,
                # Substitutions resolve to strings, so the type has to be
                # stated or the node rejects it as not a double.
                "max_actuator_speed": ParameterValue(
                    max_actuator_speed, value_type=float
                ),
            }
        ],
    )

    nodes = [bucket_driver]

    return LaunchDescription(arguments + nodes)
