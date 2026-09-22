from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command


def generate_launch_description():

    # TODO: point this to the global robot xacro/URDF, which should include
    # bucket.ros2_control.xacro (see description/bucket.ros2_control.xacro).
    robot_description_content = Command(
        [
            "xacro ",
            PathJoinSubstitution(
                [
                    FindPackageShare("payloads"),
                    "description",
                    "bucket.ros2_control.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controller_params = PathJoinSubstitution(
        [FindPackageShare("payloads"), "config", "bucket_controller.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="payloads",
        parameters=[robot_description, controller_params],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="payloads",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/payloads/controller_manager",
        ],
    )

    # Only spawn ONE of position/velocity to start with - see the note in
    # bucket_controllers.yaml about not claiming the same interface twice.
    bucket_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="payloads",
        arguments=[
            "bucket_position_controller",
            "--controller-manager",
            "/payloads/controller_manager",
        ],
    )

    delay_controller_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[bucket_controller_spawner],
        )
    )

    # NODES
    # bucket_controller = Node(
    #     package="payloads",
    #     executable="bucket_controller",
    #     output="both",
    # )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_controller_after_broadcaster,
    ]

    return LaunchDescription(nodes)
