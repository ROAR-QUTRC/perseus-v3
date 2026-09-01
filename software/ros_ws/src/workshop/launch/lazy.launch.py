from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    public_args = DeclareLaunchArgument(
        "publish_period_ms",
        default_value="500",
        description="Period in ms for publishing",
    )

    publisher_node = Node(
        package="workshop",
        executable="pub",
        output="screen",
        parameters=[
            {
                "publish_message": "Hello from launch!",
                "publish_period_ms": LaunchConfiguration("publish_period_ms"),
            }
        ],
    )

    listener_node = Node(
        package="demo_nodes_cpp",
        executable="listener",
        output="screen",
    )

    return LaunchDescription([public_args, publisher_node, listener_node])
