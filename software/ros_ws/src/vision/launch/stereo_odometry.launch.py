"""Launch the stereo visual odometry node with the shared vision configuration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Build the stereo_odometry node, overriding topics for the simulator if asked."""
    vision_dir = get_package_share_directory("vision")
    config_dir = os.path.join(vision_dir, "config")
    config_file = os.path.join(config_dir, "vision.yaml")

    parameters = [config_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}]

    if LaunchConfiguration("sim").perform(context) == "true":
        # perseus_simulation's Gazebo bridge publishes .../infra1/image_raw and
        # .../infra2/image_raw -- a synthetic pinhole camera has no distortion to
        # rectify away, so there is no "_rect" variant, unlike the real driver.
        parameters.append(
            {
                "left_image_topic": "/camera/camera/infra1/image_raw",
                "right_image_topic": "/camera/camera/infra2/image_raw",
            }
        )

    return [
        Node(
            package="vision",
            executable="stereo_odometry",
            name="stereo_odometry",
            parameters=parameters,
            output="screen",
        )
    ]


def generate_launch_description():
    """Build the launch description for the stereo_odometry node."""
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulated time"
    )
    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="false",
        description="Remap left/right image topics to the simulator's un-suffixed "
        "(no '_rect') infra1/infra2 topics. You almost always want use_sim_time:=true "
        "alongside it.",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            sim_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
