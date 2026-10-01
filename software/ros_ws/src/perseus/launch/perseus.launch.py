from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ARGUMENTS
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    hardware_plugin = LaunchConfiguration("hardware_plugin")
    can_bus = LaunchConfiguration("can_bus")
    use_wheel_pid = LaunchConfiguration("use_wheel_pid")
    safe_speed = LaunchConfiguration("safe_speed")
    min_command_erpm = LaunchConfiguration("min_command_erpm")

    arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            description="Use time provided by simulation",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="False",
            description="Use mock hardware components which mirror commands to state interfaces (overrides hardware_plugin)",
        ),
        DeclareLaunchArgument(
            "hardware_plugin",
            default_value="hardware/VescSystemHardware",
            choices=[
                "mock_components/GenericSystem",
                "hardware/VescSystemHardware",
                "gz_ros2_control/GazeboSimSystem",
            ],
            description="The hardware plugin to use for ros2_control",
        ),
        DeclareLaunchArgument(
            "can_bus",
            default_value="can0",
            description="CAN bus to use for hardware communications",
        ),
        DeclareLaunchArgument(
            "payload",
            default_value="",
            description="Which payload to boot up with the rover",
        ),
        DeclareLaunchArgument(
            "bucket_controller",
            default_value="bucket_lift_controller",
            choices=["none", "bucket_lift_controller", "bucket_trajectory_controller"],
            description=(
                "payload:=bucket only. Naming a controller commands the bucket "
                "over ros2_control and drops teleop, since the firmware follows "
                "whichever command it received last. 'none' reads the encoders "
                "for the model and leaves the bucket on gamepad teleop"
            ),
        ),
        # The two low-speed stall mitigations, both off by default so the rover
        # behaves exactly as before unless one is asked for. They are
        # independent, so they can be enabled separately to tell them apart.
        DeclareLaunchArgument(
            "safe_speed",
            default_value="true",
            description=(
                "Reduced-speed mode: caps the rover at 0.8 m/s and 0.4 rad/s. "
                "Enforced in the diff drive controller, so it binds the joystick, "
                "the web UI and nav2 alike. See perseus/config/safe_speed.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "use_wheel_pid",
            default_value="true",
            description=(
                "Chain a per-wheel velocity PID between the diff drive controller "
                "and the hardware. See perseus/config/wheel_pid_chaining.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "min_command_erpm",
            default_value="0",
            description=(
                "Raise non-zero ESC speed commands below this ERPM up to it. "
                "0 disables. A blunter alternative to use_wheel_pid: it gets the "
                "wheel moving but the actual speed no longer matches the command"
            ),
        ),
    ]

    # IMPORTED LAUNCH FILES
    def robot_state_publisher(context):
        performed_use_mock_hardware = IfCondition(use_mock_hardware).evaluate(context)
        final_hardware_plugin = (
            "mock_components/GenericSystem"
            if performed_use_mock_hardware
            else hardware_plugin
        )
        rsp_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("perseus"),
                            "launch",
                            "robot_state_publisher.launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "hardware_plugin": final_hardware_plugin,
                "can_bus": can_bus,
                "min_command_erpm": min_command_erpm,
                "payload": LaunchConfiguration("payload"),
            }.items(),
        )
        return [rsp_launch]

    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus"),
                        "launch",
                        "controllers.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_wheel_pid": use_wheel_pid,
            "safe_speed": safe_speed,
        }.items(),
    )
    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus"),
                        "launch",
                        "twist_mux.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("rosbridge_server"),
                        "launch",
                        "rosbridge_websocket_launch.xml",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    def launch_payload(context):
        payload = context.perform_substitution(LaunchConfiguration("payload"))
        if payload != "bucket":
            return []
        bucket_controller = context.perform_substitution(
            LaunchConfiguration("bucket_controller")
        )
        # Set explicitly: an included launch file sees this file's launch
        # configurations, so without it bucket.launch.py picks up the drive's
        # hardware_plugin (hardware/VescSystemHardware) and its controller
        # manager never starts.
        bucket_hardware = (
            "payloads/BucketHardware"
            if context.perform_substitution(hardware_plugin)
            == "hardware/VescSystemHardware"
            and not IfCondition(use_mock_hardware).evaluate(context)
            else "mock_components/GenericSystem"
        )
        actions = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [FindPackageShare("payloads"), "launch", "bucket.launch.py"]
                        )
                    ]
                ),
                launch_arguments={
                    "hardware_plugin": bucket_hardware,
                    "can_interface": can_bus,
                    "controller": bucket_controller,
                }.items(),
            ),
            # The bucket has its own controller manager, so its joints arrive on
            # /payloads/joint_states. robot_state_publisher reads /joint_states,
            # so forward them there with the ten ram joints solved from lift,
            # tilt and jaw. The drive's broadcaster keeps the wheel joints.
            Node(
                package="description",
                executable="bucket_ram_follower.py",
                name="bucket_ram_follower",
                parameters=[
                    {
                        "viewer_mode": True,
                        "input_topic": "/payloads/joint_states",
                        "output_topic": "/joint_states",
                        "hold_wheels": False,
                    }
                ],
                output="both",
            ),
        ]
        # Calibration mode claims no command interface, so teleop can run
        # alongside it. With a controller active it must not.
        if bucket_controller == "none":
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("payloads"),
                                    "launch",
                                    "bucket_teleop.launch.py",
                                ]
                            )
                        ]
                    ),
                    launch_arguments={"can_bus": can_bus}.items(),
                )
            )
        return actions

    launch_files = [
        OpaqueFunction(function=robot_state_publisher),
        controllers_launch,
        twist_mux_launch,
        rosbridge_launch,
        OpaqueFunction(function=launch_payload),
    ]

    return LaunchDescription(arguments + launch_files)
