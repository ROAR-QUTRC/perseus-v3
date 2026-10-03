"""Autonomous (ros2_control) control of the excavation bucket.

Brings up a controller_manager in the `payloads` namespace driving the bucket
over CAN with SET_POSITION, letting the board's own PID servo to each setpoint.

Deliberately kept separate from bucket_teleop.launch.py: the firmware picks its
control mode from whichever command it received last, so if both stacks
transmit at once each bank flips between open-loop speed and closed-loop
position every frame. controller:=teleop is the one safe way to combine them:
no command controller is spawned, so the hardware only reads the encoders and
bucket_teleop.launch.py is included to drive the bucket from the gamepad.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

NAMESPACE = "payloads"


def generate_launch_description():
    # ARGUMENTS
    can_interface = LaunchConfiguration("can_interface", default="can0")
    hardware_plugin = LaunchConfiguration("hardware_plugin")
    controller = LaunchConfiguration("controller")

    arguments = [
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="CAN interface the bucket controller is on",
        ),
        DeclareLaunchArgument(
            "hardware_plugin",
            default_value="payloads/BucketHardware",
            description="ros2_control hardware plugin; mock_components/GenericSystem runs without the bucket",
        ),
        DeclareLaunchArgument(
            "controller",
            default_value="bucket_trajectory_controller",
            description=(
                "Command controller to spawn: bucket_trajectory_controller (all "
                "axes), bucket_lift_controller, bucket_tilt_controller or "
                "bucket_jaw_controller (one axis each), none (read-only "
                "calibration mode), or teleop (read-only plus the gamepad "
                "bucket_driver)"
            ),
        ),
    ]

    # CONFIG + DATA FILES
    robot_description_content = Command(
        [
            "xacro ",
            PathJoinSubstitution(
                [
                    FindPackageShare("payloads"),
                    "urdf",
                    "bucket_standalone.urdf.xacro",
                ]
            ),
            " can_interface:=",
            can_interface,
            " hardware_plugin:=",
            hardware_plugin,
        ]
    )
    # Must be declared a string, or the xacro output is passed as a
    # substitution object and the node rejects the parameter.
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    controller_params = PathJoinSubstitution(
        [FindPackageShare("payloads"), "config", "bucket_controller.yaml"]
    )

    # NODES
    # Jazzy's controller_manager reads robot_description from a *topic*, not a
    # parameter, so something has to publish it in this namespace or the manager
    # waits forever at "Waiting for data on 'robot_description' topic".
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=NAMESPACE,
        parameters=[robot_description],
        # Only here to feed controller_manager its robot_description. Its TF is
        # kept off /tf so it cannot fight the main robot_state_publisher, which
        # owns the bucket frames.
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        output="both",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=NAMESPACE,
        parameters=[robot_description, controller_params],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=NAMESPACE,
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            f"/{NAMESPACE}/controller_manager",
        ],
    )

    # controller:=none is calibration mode: nothing claims a command interface, so
    # the hardware only reads and bucket_driver teleop can move the bucket.
    # controller:=teleop is the same, with that teleop started here too.
    bucket_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=NAMESPACE,
        arguments=[
            controller,
            "--controller-manager",
            f"/{NAMESPACE}/controller_manager",
        ],
        condition=IfCondition(
            PythonExpression(["'", controller, "' not in ('none', 'teleop')"])
        ),
    )

    bucket_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("payloads"), "launch", "bucket_teleop.launch.py"]
            )
        ),
        launch_arguments={"can_bus": can_interface}.items(),
        condition=IfCondition(PythonExpression(["'", controller, "' == 'teleop'"])),
    )

    joint_states_deg = Node(
        package="description",
        executable="joint_states_deg.py",
        namespace=NAMESPACE,
        output="both",
    )

    # EVENT HANDLERS
    # Spawn the command controller only once the broadcaster is up, so a failure
    # to read encoders surfaces before anything can be commanded.
    delay_controller_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[bucket_controller_spawner],
        )
    )

    nodes = [
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        joint_states_deg,
        bucket_teleop,
    ]

    return LaunchDescription(arguments + nodes + [delay_controller_after_broadcaster])
