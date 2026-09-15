"""Full arm on real servos: hardware plugin, IKFast, MoveIt Servo, Joy teleop.

Everything servo_sim.launch.py runs, but driving the Dynamixel bus instead of
mock hardware. Defaults to servos 1-3 (shoulder_pan, shoulder_tilt, elbow) live
and 4-6 (wrist, gripper) simulated, so the 3-DOF Cartesian group can be flown
before the wrist and gripper are wired.

    ros2 launch payloads arm_hardware.launch.py

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Wrist (4, 5) and gripper (6) are simulated until they are wired and, for the
# gripper, until prismatic_scale in config/arm.ros2_control.xacro is measured.
MOCK_SERVO_IDS = "4,5,6"


def generate_launch_description():
    arm_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("payloads"), "launch", "arm_teleop_sim.launch.py"]
            )
        ),
        launch_arguments={
            "use_mock_hardware": "false",
            "device": LaunchConfiguration("device"),
            "mock_servo_ids": LaunchConfiguration("mock_servo_ids"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "joy_topic": LaunchConfiguration("joy_topic"),
            "joy_timeout": LaunchConfiguration("joy_timeout"),
            "command_frame": LaunchConfiguration("command_frame"),
        }.items(),
    )

    return LaunchDescription(
        [
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
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2",
            ),
            DeclareLaunchArgument(
                "joy_topic",
                default_value="/joy",
                description="DualSense Joy input topic",
            ),
            DeclareLaunchArgument(
                "joy_timeout",
                default_value="0.25",
                description="Seconds before stale Joy input locks teleoperation",
            ),
            DeclareLaunchArgument(
                "command_frame",
                default_value="plate",
                description="Frame used for Cartesian Servo commands",
            ),
            arm_teleop,
        ]
    )
