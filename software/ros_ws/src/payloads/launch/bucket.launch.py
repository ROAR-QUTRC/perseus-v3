from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    # NODES
    bucket_driver = Node(
        package="payloads",
        executable="bucket_driver",
        output="both",
    )
    bucket_controller = Node(
        package="payloads",
        executable="bucket_controller",
        output="both",
    )

    nodes = [bucket_driver, bucket_controller]

    return LaunchDescription(nodes)
