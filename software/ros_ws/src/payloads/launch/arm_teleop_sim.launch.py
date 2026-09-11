from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    joy_topic = LaunchConfiguration("joy_topic")
    command_frame = LaunchConfiguration("command_frame")
    joy_timeout = LaunchConfiguration("joy_timeout")

    servo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("payloads"), "launch", "servo_sim.launch.py"]
            )
        ),
        launch_arguments={
            "use_rviz": use_rviz,
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
            servo_sim,
            joy_node,
            arm_teleop,
        ]
    )
