from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    bucket = context.perform_substitution(LaunchConfiguration("payload")) == "bucket"

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
    if bucket:
        controller_parameters.append(
            PathJoinSubstitution(
                [FindPackageShare("payloads"), "config", "bucket_controller.yaml"]
            )
        )
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

    if bucket:
        nodes += bucket_nodes(context, use_sim_time_param)

    return nodes + handlers


def bucket_nodes(context, use_sim_time_param):
    """The bucket's controllers, spawned beside the drive's under the one manager.

    Its own chain, not hung off the drive's OnProcessExit handlers, so a bucket that
    fails to come up (no encoders, no board) can never hold up the drive.
    """
    bucket_controller = context.perform_substitution(
        LaunchConfiguration("bucket_controller")
    )
    real = (
        context.perform_substitution(LaunchConfiguration("bucket_hardware_plugin"))
        == "payloads/BucketHardware"
    )
    teleop = real and IfCondition(LaunchConfiguration("bucket_teleop")).evaluate(
        context
    )

    # Read-only, so they run whatever the bucket is doing: the encoder angles and
    # the ram joints solved from them, both on /joint_states.
    broadcasters_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bucket_joint_state_broadcaster", "bucket_linkage_broadcaster"],
        parameters=[use_sim_time_param],
    )
    nodes = [
        broadcasters_spawner,
        Node(
            package="description",
            executable="joint_states_deg.py",
            parameters=[
                {
                    "joints": [
                        "bucket_lift_joint",
                        "bucket_tilt_joint",
                        "bucket_jaw_joint",
                    ]
                },
                use_sim_time_param,
            ],
            output="both",
        ),
    ]
    if bucket_controller != "none":
        # After the broadcasters, so a failure to read the encoders surfaces before
        # anything can be commanded. BucketHardware refuses the claim outright
        # while its encoders are stale or out of range.
        nodes.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=broadcasters_spawner,
                    on_exit=[
                        Node(
                            package="controller_manager",
                            executable="spawner",
                            arguments=[bucket_controller],
                            parameters=[use_sim_time_param],
                        )
                    ],
                )
            )
        )
    if real:
        # Brings the bucket's read-only side back after a fault; never re-arms the
        # command controller by itself.
        nodes.append(
            Node(
                package="payloads",
                executable="bucket_supervisor.py",
                output="both",
            )
        )
    if teleop:
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("payloads"),
                            "launch",
                            "bucket_teleop.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "can_bus": LaunchConfiguration("can_bus"),
                }.items(),
            )
        )
    return nodes


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

    bucket_arguments = [
        DeclareLaunchArgument(
            "payload",
            default_value="",
            description="'bucket' adds the bucket's controllers to this manager",
        ),
        DeclareLaunchArgument(
            "bucket_controller",
            default_value="bucket_trajectory_controller",
            description="See perseus.launch.py",
        ),
        DeclareLaunchArgument(
            "bucket_hardware_plugin",
            default_value="payloads/BucketHardware",
            description="The bucket's ros2_control plugin, as put in the URDF",
        ),
        DeclareLaunchArgument(
            "bucket_teleop",
            default_value="true",
            description="See perseus.launch.py",
        ),
        DeclareLaunchArgument(
            "can_bus",
            default_value="can0",
            description="CAN bus for the bucket teleop driver",
        ),
    ]

    return LaunchDescription(
        arguments + bucket_arguments + [OpaqueFunction(function=launch_setup)]
    )
