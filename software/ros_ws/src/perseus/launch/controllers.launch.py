from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_controller_manager = LaunchConfiguration("launch_controller_manager")
    use_sim_time_param = {"use_sim_time": use_sim_time}

    # Resolved here rather than passed as a condition because it changes the
    # SHAPE of the launch (an extra spawner, and a different chain of event
    # handlers), not just whether one node runs.
    use_wheel_pid = IfCondition(LaunchConfiguration("use_wheel_pid")).evaluate(context)
    safe_speed = IfCondition(LaunchConfiguration("safe_speed")).evaluate(context)

    # CONFIG + DATA FILES
    controller_config = PathJoinSubstitution(
        [FindPackageShare("perseus"), "config", "perseus_controllers.yaml"]
    )
    # Overlay, so it must come after the base file for its overrides to win.
    wheel_pid_config = PathJoinSubstitution(
        [FindPackageShare("perseus"), "config", "wheel_pid_chaining.yaml"]
    )
    safe_speed_config = PathJoinSubstitution(
        [FindPackageShare("perseus"), "config", "safe_speed.yaml"]
    )

    # Order matters: later files win per parameter. The two overlays touch
    # disjoint keys of diff_drive_base_controller (wheel names and feedback
    # source vs velocity ceilings), so they compose and can be used together.
    controller_parameters = [controller_config]
    if use_wheel_pid:
        controller_parameters.append(wheel_pid_config)
    if safe_speed:
        controller_parameters.append(safe_speed_config)
    controller_parameters.append(use_sim_time_param)

    # NODES
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=controller_parameters,
        output="both",  # output to both screen and log file
        remappings=[],
        condition=IfCondition(launch_controller_manager),
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
        ],
        parameters=[use_sim_time_param],
    )
    base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_base_controller",
            "--controller-ros-args",
            "--remap /diff_drive_base_controller/cmd_vel:=/cmd_vel_out --remap /diff_drive_base_controller/odom:=/odom",
        ],
        parameters=[use_sim_time_param],
    )
    # The following half of the chain. It has to be active before the diff drive
    # controller starts, because that is what claims its reference interfaces
    # and switches it from standalone into chained mode.
    wheel_pid_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wheel_pid_controller",
        ],
        parameters=[use_sim_time_param],
    )

    # NOTE: There was a comment in one of the ROS2 Control examples
    # about launching the controllers *after* the controller manager
    # to help with "flaky tests" (ie, using RegisterEventHandler with OnProcessExit)
    # to launch them in sequence
    nodes = [
        controller_manager,
        joint_state_broadcaster_spawner,
    ]

    # EVENT HANDLERS
    if use_wheel_pid:
        handlers = [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[wheel_pid_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=wheel_pid_spawner,
                    on_exit=[base_controller_spawner],
                )
            ),
        ]
    else:
        handlers = [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[base_controller_spawner],
                )
            ),
        ]

    return nodes + handlers


def generate_launch_description():
    # ARGUMENTS
    arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Use time provided by simulation",
        ),
        DeclareLaunchArgument(
            "launch_controller_manager",
            default_value="true",
            description="Launch the controller manager (off when something else owns it, eg Gazebo)",
        ),
        DeclareLaunchArgument(
            "safe_speed",
            default_value="true",
            description=(
                "Cap the rover at 0.8 m/s and 0.4 rad/s in the diff drive "
                "controller's speed limiter, so every command source is limited "
                "at once. See config/safe_speed.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "use_wheel_pid",
            default_value="true",
            description=(
                "Chain a per-wheel velocity PID between the diff drive controller "
                "and the hardware, to push through low-speed stall. See "
                "config/wheel_pid_chaining.yaml"
            ),
        ),
    ]

    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])
