"""Launch the camera and the vision pipeline as one stack.

This is the entry point for running vision on the vehicle: it brings up the RealSense
driver from `sensors/launch/realsense.launch.py` and the vision nodes that consume its
colour stream.

Each node runs as its own process, the same way the single-node launch files run them,
so one node crashing or being restarted does not take the others down with it.

Which nodes run is controlled per node. The overlay is on by default because it is cheap
and is what makes the stream viewable; both detectors are off, since each costs real CPU
and is only wanted when something is actually consuming its detections:

    overlay        detection overlay          (default true)
    aruco_detect   ArUco marker detector      (default false)
    cube_detect    YOLO cube detector         (default false)

Arguments declared by realsense.launch.py (enable_depth, enable_infra_pair,
enable_color) pass straight through.

Usage:
    ros2 launch vision vision.launch.py

    ros2 launch vision vision.launch.py aruco_detect:=true cube_detect:=true

    ros2 launch vision vision.launch.py aruco_detect:=true enable_depth:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the launch description for the camera and the vision pipeline."""
    vision_dir = get_package_share_directory("vision")
    config_dir = os.path.join(vision_dir, "config")
    config_file = os.path.join(config_dir, "vision.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulated time"
    )
    overlay_arg = DeclareLaunchArgument(
        "overlay",
        default_value="true",
        description="Run the detection overlay node",
    )
    aruco_detect_arg = DeclareLaunchArgument(
        "aruco_detect",
        default_value="false",
        description="Run the ArUco marker detector",
    )
    cube_detect_arg = DeclareLaunchArgument(
        "cube_detect",
        default_value="false",
        description="Run the cube detector",
    )

    # Not wrapped in a scoped group: realsense.launch.py declares its own stream
    # toggles, and leaving the include unscoped is what lets them be set from this
    # launch file's command line.
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sensors"), "launch", "realsense.launch.py"]
            )
        )
    )

    common_parameters = [
        config_file,
        {"use_sim_time": LaunchConfiguration("use_sim_time")},
    ]

    def vision_node(executable, name, argument):
        return Node(
            package="vision",
            executable=executable,
            name=name,
            parameters=common_parameters,
            output="screen",
            condition=IfCondition(LaunchConfiguration(argument)),
        )

    return LaunchDescription(
        [
            use_sim_time_arg,
            overlay_arg,
            aruco_detect_arg,
            cube_detect_arg,
            realsense,
            vision_node("detection_overlay_node", "detection_overlay", "overlay"),
            vision_node("aruco_detector_node", "aruco_detector", "aruco_detect"),
            vision_node("cube_detector", "cube_detector", "cube_detect"),
        ]
    )
