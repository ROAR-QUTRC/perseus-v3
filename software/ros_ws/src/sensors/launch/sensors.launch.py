"""Launch the full sensor pack (Livox LiDAR + RealSense camera).

Usage:
    ros2 launch sensors sensor_pack.launch.py

    ros2 launch sensors sensor_pack.launch.py realsense:=false
    ros2 launch sensors sensor_pack.launch.py livox:=false
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

SENSORS_LAUNCH_DIR = Path(get_package_share_directory("sensors")) / "launch"


def generate_launch_description():
    """Build the launch description for the full sensor pack."""
    realsense_arg = DeclareLaunchArgument(
        "realsense",
        default_value="true",
        description="Enable the RealSense camera launch",
    )
    livox_arg = DeclareLaunchArgument(
        "livox",
        default_value="true",
        description="Enable the Livox LiDAR launch",
    )

    livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(SENSORS_LAUNCH_DIR / "livox.launch.py")),
        condition=IfCondition(LaunchConfiguration("livox")),
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(SENSORS_LAUNCH_DIR / "realsense.launch.py")),
        condition=IfCondition(LaunchConfiguration("realsense")),
    )

    return LaunchDescription(
        [
            realsense_arg,
            livox_arg,
            livox,
            realsense,
        ]
    )
