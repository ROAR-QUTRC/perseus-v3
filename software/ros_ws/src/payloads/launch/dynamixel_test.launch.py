"""Bench test for the payloads/DynamixelServos hardware plugin, alone.

Brings up only ros2_control on the real Dynamixel bus -- no MoveIt, no Servo,
no RViz -- so a plugin or wiring problem is not hidden behind the rest of the
stack. Defaults to servos 1-3 live and 4-6 simulated.

    ros2 launch payloads dynamixel_test.launch.py

Expect, in order: "Mocking 3 servo ID(s)", a successful scan, then
"Configured and activated joint_state_broadcaster" and "... servo_controller".
Torque engages on activation, so the arm holds position -- support it first.

Confirm the servos are talking:

    ros2 topic echo /joint_states            # positions track the real arm
    cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer   # expect 1

Move one joint (radians, 3 s):

    ros2 topic pub --once /servo_controller/joint_trajectory \
      trajectory_msgs/msg/JointTrajectory \
      '{joint_names: [shoulder_pan, shoulder_tilt, elbow],
        points: [{positions: [0.3, -0.75, 1.7], time_from_start: {sec: 3}}]}'

Other ports or a different live/mock split:

    ros2 launch payloads dynamixel_test.launch.py device:=/dev/ttyUSB1
    ros2 launch payloads dynamixel_test.launch.py mock_servo_ids:=2,3,4,5,6
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

HARDWARE_PLUGIN = "payloads/DynamixelServos"
MOCK_SERVO_IDS = "4,5,6"


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "hardware_plugin",
            default_value=HARDWARE_PLUGIN,
            description="ros2_control hardware_interface plugin under test",
        ),
        DeclareLaunchArgument(
            "device",
            default_value="/dev/ttyUSB0",
            description="Serial port the Dynamixel bus is attached to",
        ),
        DeclareLaunchArgument(
            "mock_servo_ids",
            default_value=MOCK_SERVO_IDS,
            description=(
                "Comma-separated servo IDs to simulate instead of "
                "communicating with the physical bus. No spaces."
            ),
        ),
    ]

    hardware_plugin = LaunchConfiguration("hardware_plugin")
    device = LaunchConfiguration("device")
    mock_servo_ids = LaunchConfiguration("mock_servo_ids")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("payloads"), "config", "arm.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=false",
            " ",
            "hardware_plugin:=",
            hardware_plugin,
            " ",
            "device:=",
            device,
            " ",
            "mock_servo_ids:=",
            mock_servo_ids,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    ros2_controllers_path = os.path.join(
        get_package_share_directory("payloads"), "config", "ros2_controllers.yaml"
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_robot_description_topic": False}],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[("robot_description", "/robot_description")],
        output="both",
    )

    def spawner(controller):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                controller,
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                "20",
                "--service-call-timeout",
                "10",
                "--switch-timeout",
                "10",
            ],
        )

    joint_state_broadcaster_spawner = spawner("joint_state_broadcaster")
    servo_controller_spawner = spawner("servo_controller")

    # The broadcaster must be up before any controller claims an interface; a
    # failed spawner means the bus never came up, so stop rather than continue.
    start_servo_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=lambda event, _context: (
                [
                    LogInfo(msg="ERROR: joint_state_broadcaster failed"),
                    EmitEvent(event=Shutdown(reason="joint_state_broadcaster failed")),
                ]
                if event.returncode != 0
                else [servo_controller_spawner]
            ),
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            start_servo_controller,
        ]
    )
