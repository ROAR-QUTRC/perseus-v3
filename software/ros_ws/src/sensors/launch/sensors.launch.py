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
    declare_interface = DeclareLaunchArgument(
        "interface",
        default_value="enP3p49s0",
        description="Network interface the Livox driver reads the host IP from, forwarded "
        "to livox.launch.py and used only when enable_sensors:=true. The default matches "
        "livox.launch.py's own; the driver fails at launch if the interface it names has no "
        "IPv4 address, so this needs to be right per machine.",
    )
    livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(SENSORS_LAUNCH_DIR / "livox.launch.py")),
        condition=IfCondition(LaunchConfiguration("livox")),
        launch_arguments={"interface": LaunchConfiguration("interface")}.items(),
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(SENSORS_LAUNCH_DIR / "realsense.launch.py")),
        condition=IfCondition(LaunchConfiguration("realsense")),
    )

    return LaunchDescription(
        [
            realsense_arg,
            livox_arg,
            declare_interface,
            livox,
            realsense,
        ]
    )
