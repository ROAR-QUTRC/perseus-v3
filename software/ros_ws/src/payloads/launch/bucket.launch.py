"""Bench bring-up of the excavation bucket on its own, without the drive.

On the rover the bucket is a second hardware component under the drive's
controller_manager (perseus.launch.py payload:=bucket), and this file is not used.
Here it gets a controller_manager of its own, with the same controllers, names and
topics as on the rover, so everything tested here (bucket_cli, the vcan simulator)
works there unchanged. Never run it alongside perseus.launch.py: both would be
/controller_manager.

The gamepad teleop (bucket_driver) runs alongside as an operator override when the
hardware is real: a stick input deactivates the command controller, and
`ros2 run payloads bucket_supervisor.py --rearm` hands control back.
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


def generate_launch_description():
    # ARGUMENTS
    can_interface = LaunchConfiguration("can_interface", default="can0")
    hardware_plugin = LaunchConfiguration("hardware_plugin")
    controller = LaunchConfiguration("controller")
    teleop = LaunchConfiguration("teleop")

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
            choices=[
                "none",
                "bucket_trajectory_controller",
                "bucket_lift_controller",
                "bucket_tilt_controller",
                "bucket_jaw_controller",
            ],
            description=(
                "Command controller to spawn: bucket_trajectory_controller (all "
                "axes), bucket_lift_controller, bucket_tilt_controller or "
                "bucket_jaw_controller (one axis each), or none (read-only "
                "calibration mode)"
            ),
        ),
        DeclareLaunchArgument(
            "teleop",
            default_value="true",
            description="Run the gamepad bucket_driver as an operator override (real hardware only)",
        ),
    ]

    # The real-hardware-only nodes: both talk to the CAN bus or to BucketHardware.
    real_hardware = PythonExpression(
        ["'", hardware_plugin, "' == 'payloads/BucketHardware'"]
    )

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
    # parameter, so something has to publish it or the manager waits forever at
    # "Waiting for data on 'robot_description' topic". Also publishes the bucket
    # frames, so the bucket shows up in RViz on its own.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # On the rover the drive's 50 Hz applies and the bucket runs at its rw_rate.
        parameters=[controller_params, {"update_rate": 20}],
        output="both",
    )

    broadcasters_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bucket_joint_state_broadcaster", "bucket_linkage_broadcaster"],
    )

    # controller:=none is calibration mode: nothing claims a command interface, so
    # the hardware only reads and the teleop moves the bucket.
    bucket_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller],
        condition=IfCondition(PythonExpression(["'", controller, "' != 'none'"])),
    )

    bucket_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("payloads"), "launch", "bucket_teleop.launch.py"]
            )
        ),
        launch_arguments={"can_bus": can_interface}.items(),
        condition=IfCondition(
            PythonExpression(["(", real_hardware, ") and ('", teleop, "' == 'true')"])
        ),
    )

    bucket_supervisor = Node(
        package="payloads",
        executable="bucket_supervisor.py",
        output="both",
        condition=IfCondition(real_hardware),
    )

    joint_states_deg = Node(
        package="description",
        executable="joint_states_deg.py",
        parameters=[
            {"joints": ["bucket_lift_joint", "bucket_tilt_joint", "bucket_jaw_joint"]}
        ],
        output="both",
    )

    # EVENT HANDLERS
    # Spawn the command controller only once the broadcasters are up, so a failure
    # to read encoders surfaces before anything can be commanded.
    delay_controller_after_broadcasters = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=broadcasters_spawner,
            on_exit=[bucket_controller_spawner],
        )
    )

    nodes = [
        robot_state_publisher,
        control_node,
        broadcasters_spawner,
        joint_states_deg,
        bucket_teleop,
        bucket_supervisor,
    ]

    return LaunchDescription(arguments + nodes + [delay_controller_after_broadcasters])
