from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("camera_device", default_value="/dev/video0",
                              description="V4L2 device path (e.g. /dev/video2) or index"),
        DeclareLaunchArgument("model_path", default_value="empty_model.pt"),
        DeclareLaunchArgument("rate_hz", default_value="2.0"),
        DeclareLaunchArgument("threshold_override", default_value="0.0"),
        Node(
            package="bucket_empty",  # change if your package has another name
            executable="bucket_empty_node",
            name="bucket_empty_detector",
            output="screen",
            parameters=[{
                "camera_device": LaunchConfiguration("camera_device"),
                "model_path": LaunchConfiguration("model_path"),
                "rate_hz": LaunchConfiguration("rate_hz"),
                "threshold_override": LaunchConfiguration("threshold_override"),
            }],
        ),
    ])