from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    device = LaunchConfiguration("device")
    mock_servo_ids = LaunchConfiguration("mock_servo_ids")
    joy_topic = LaunchConfiguration("joy_topic")
    command_frame = LaunchConfiguration("command_frame")
    joy_timeout = LaunchConfiguration("joy_timeout")
    rmw_implementation = LaunchConfiguration("rmw_implementation")

    servo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("payloads"), "launch", "servo_sim.launch.py"]
            )
        ),
        launch_arguments={
            "use_rviz": use_rviz,
            "use_mock_hardware": use_mock_hardware,
            "device": device,
            "mock_servo_ids": mock_servo_ids,
            "rmw_implementation": rmw_implementation,
        }.items(),
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        remappings=[("joy", joy_topic)],
    )

    arm_teleop = Node(
        package="teleop",
        executable="arm_teleop",
        name="arm_teleop",
        output="screen",
        parameters=[
            {
                "command_frame": command_frame,
                "joy_timeout": joy_timeout,
            }
        ],
        remappings=[
            ("joy", joy_topic),
            ("command/end_effector_twist", "/servo_node/delta_twist_cmds"),
            ("command/shoulder_joint_jog", "/servo_node/delta_joint_cmds"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2",
            ),
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="true",
                description="Mirror commands to states instead of driving the bus",
            ),
            DeclareLaunchArgument(
                "device",
                default_value="/dev/ttyUSB0",
                description="Serial port the Dynamixel bus is attached to",
            ),
            DeclareLaunchArgument(
                "mock_servo_ids",
                default_value="4,5,6",
                description=(
                    "Comma-separated servo IDs to simulate instead of "
                    "communicating with the physical bus. No spaces."
                ),
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
            DeclareLaunchArgument(
                "rmw_implementation",
                default_value="rmw_cyclonedds_cpp",
                description="ROS middleware used by all simulation and teleop processes",
            ),
            SetEnvironmentVariable(
                name="RMW_IMPLEMENTATION",
                value=rmw_implementation,
            ),
            SetEnvironmentVariable(
                name="CYCLONEDDS_URI",
                value="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name='lo'/></Interfaces><AllowMulticast>false</AllowMulticast></General></Domain></CycloneDDS>",
            ),
            servo_sim,
            joy_node,
            arm_teleop,
        ]
    )
