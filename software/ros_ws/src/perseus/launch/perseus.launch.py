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
            default_value="bucket_trajectory_controller",
            choices=[
                "none",
                "bucket_trajectory_controller",
                "bucket_lift_controller",
                "bucket_tilt_controller",
                "bucket_jaw_controller",
            ],
            description=(
                "payload:=bucket only. The controller that commands the bucket "
                "over ros2_control. 'none' only reads the encoders for the model. "
                "The gamepad teleop runs alongside either way (bucket_teleop) and "
                "takes over from this controller when a stick moves"
            ),
        ),
        DeclareLaunchArgument(
            "bucket_hardware",
            default_value="real",
            choices=["real", "mock"],
            description=(
                "payload:=bucket only. 'mock' runs the bucket on "
                "mock_components/GenericSystem with the real drive. The bucket is "
                "mocked anyway whenever the drive is"
            ),
        ),
        DeclareLaunchArgument(
            "bucket_teleop",
            default_value="true",
            description=(
                "payload:=bucket only. Run the gamepad bucket_driver as an operator "
                "override over the bucket controller. Only with real bucket hardware"
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

    def bucket_hardware_plugin(context):
        """The bucket's ros2_control plugin: real only alongside the real drive."""
        drive_is_real = context.perform_substitution(
            hardware_plugin
        ) == "hardware/VescSystemHardware" and not IfCondition(
            use_mock_hardware
        ).evaluate(context)
        bucket_is_real = (
            context.perform_substitution(LaunchConfiguration("bucket_hardware"))
            == "real"
        )
        if drive_is_real and bucket_is_real:
            return "payloads/BucketHardware"
        return "mock_components/GenericSystem"

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
                "bucket_hardware_plugin": bucket_hardware_plugin(context),
            }.items(),
        )
        return [rsp_launch]

    # The bucket, when fitted, is a second hardware component under this same
    # controller_manager (see perseus.urdf.xacro), so its controllers are spawned
    # from here too.
    def controllers(context):
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
                "payload": LaunchConfiguration("payload"),
                "bucket_controller": LaunchConfiguration("bucket_controller"),
                "bucket_hardware_plugin": bucket_hardware_plugin(context),
                "bucket_teleop": LaunchConfiguration("bucket_teleop"),
                "can_bus": can_bus,
            }.items(),
        )
        return [controllers_launch]

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

    launch_files = [
        OpaqueFunction(function=robot_state_publisher),
        OpaqueFunction(function=controllers),
        twist_mux_launch,
        rosbridge_launch,
    ]

    return LaunchDescription(arguments + launch_files)
