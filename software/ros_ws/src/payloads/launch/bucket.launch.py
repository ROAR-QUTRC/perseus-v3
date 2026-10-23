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

sim:=true is the whole stack against a simulated bucket board: rover_can_sim.py on
vcan0, with the real BucketHardware and bucket_driver talking to it, so teleop,
zeroing, the trajectory controller and bucket_teach.py all work without the bucket.
vcan0 must exist first (software/scripts/vcan-setup.sh). The simulator runs headless
here; to inject faults, run it yourself instead and launch with can_interface:=vcan0.

rviz:=true shows the bucket in RViz (through nixGL, as description's view_perseus
does); by default it opens with sim:=true and stays shut on real hardware, which is
usually reached over SSH.
"""

import os


from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
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

SIM_INTERFACE = "vcan0"


def _check_sim_interface(context):
    """Stops the launch with the fix, rather than leaving every node failing to open it."""
    if context.perform_substitution(LaunchConfiguration("sim")) != "true":
        return []
    if not os.path.exists(f"/sys/class/net/{SIM_INTERFACE}"):
        raise RuntimeError(
            f"sim:=true needs {SIM_INTERFACE}: run software/scripts/vcan-setup.sh first"
        )
    return []


def generate_launch_description():
    # ARGUMENTS
    can_interface = LaunchConfiguration("can_interface", default="can0")
    hardware_plugin = LaunchConfiguration("hardware_plugin")
    controller = LaunchConfiguration("controller")
    teleop = LaunchConfiguration("teleop")
    sim = LaunchConfiguration("sim")
    rviz = LaunchConfiguration("rviz")

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
        DeclareLaunchArgument(
            "sim",
            default_value="false",
            choices=["true", "false"],
            description="Simulate the bucket board with rover_can_sim.py on vcan0 "
            "(overrides can_interface)",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="auto",
            choices=["auto", "true", "false"],
            description="Show the bucket in RViz; auto opens it only with sim:=true",
        ),
    ]

    # sim:=true always means vcan0, whatever can_interface says.
    can_bus = PythonExpression(
        ["'", SIM_INTERFACE, "' if '", sim, "' == 'true' else '", can_interface, "'"]
    )

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
            can_bus,
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
        launch_arguments={"can_bus": can_bus}.items(),
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

    # Started first, so the encoders are broadcasting before the command controller
    # is spawned (BucketHardware refuses a controller while they are stale).
    bucket_board_sim = Node(
        package="payloads",
        executable="rover_can_sim.py",
        arguments=["--iface", SIM_INTERFACE, "--no-drive", "--status-period", "0"],
        output="both",
        condition=IfCondition(sim),
    )

    joint_states_deg = Node(
        package="description",
        executable="joint_states_deg.py",
        parameters=[
            {"joints": ["bucket_lift_joint", "bucket_tilt_joint", "bucket_jaw_joint"]}
        ],
        output="both",
    )

    # RViz with nixGL support, as description's view_perseus.launch.py runs it.
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("payloads"), "rviz", "view_bucket.rviz"]
    )
    rviz_gui = ExecuteProcess(
        cmd=[
            "nix",
            "run",
            "--impure",
            "github:nix-community/nixGL",
            "--",
            "rviz2",
            "-d",
            rviz_config,
        ],
        output="screen",
        additional_env={
            "NIXPKGS_ALLOW_UNFREE": "1",
            "QT_QPA_PLATFORM": "xcb",
            "QT_SCREEN_SCALE_FACTORS": "1",
            "ROS_NAMESPACE": "/",
            "RMW_QOS_POLICY_HISTORY": "keep_last",
            "RMW_QOS_POLICY_DEPTH": "100",
        },
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    rviz,
                    "' == 'true' or ('",
                    rviz,
                    "' == 'auto' and '",
                    sim,
                    "' == 'true')",
                ]
            )
        ),
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
        OpaqueFunction(function=_check_sim_interface),
        bucket_board_sim,
        robot_state_publisher,
        control_node,
        broadcasters_spawner,
        joint_states_deg,
        bucket_teleop,
        bucket_supervisor,
        rviz_gui,
    ]

    return LaunchDescription(arguments + nodes + [delay_controller_after_broadcasters])
