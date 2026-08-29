"""Launch arena_server with the arena layout.

The layout lives in config/arena_layout.yaml rather than in this file so the
simulation and the autonomy stack can read the same numbers. Guidebook sections
3.1 and 5.7 both say the arena layout is provisional, so expect to re-measure
into that one file rather than hunting constants through the code.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    layout = LaunchConfiguration("layout")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "layout",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("arena_server"),
                        "config",
                        "arena_layout.yaml",
                    ]
                ),
                description="Arena zone and fiducial layout to load",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="If true, use the simulated clock",
            ),
            Node(
                package="arena_server",
                executable="arena_server",
                name="arena_server",
                output="screen",
                parameters=[layout, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
