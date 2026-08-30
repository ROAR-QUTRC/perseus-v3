"""Base-station minimap: draws the arena locally, tracks the rover pose.

The point of this node is what it does NOT need. It reads the same
arena_layout.json the robot reads and draws the arena from that local copy, so
nothing about the map crosses the link -- no costmap, no mesh, no map server.
The only thing that comes over the network is /arena/robot_pose, one small
PoseStamped at a few Hz.

Both ends log their layout summary at startup. If those two lines disagree, the
base station is drawing an arena the robot is not navigating in, and nothing
else will tell you.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    layout_json = LaunchConfiguration("layout_json")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "layout_json",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("autonomy_bringup"),
                        "config",
                        "arena_layout.json",
                    ]
                ),
                description="Arena geometry; must be the same file the robot reads",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description=(
                    "Base station usually runs on wall time. Set true only if it "
                    "shares the robot's simulated clock."
                ),
            ),
            Node(
                package="arena_server",
                executable="arena_minimap",
                name="arena_minimap",
                output="screen",
                parameters=[
                    {
                        "layout_file": layout_json,
                        "use_sim_time": use_sim_time,
                        # Flat-ish walls: this is a top-down minimap, so tall
                        # zone walls would occlude the rover marker rather than
                        # frame it.
                        "zone_height_m": 0.20,
                        "zone_alpha": 0.35,
                        "robot_marker_radius_m": 0.35,
                        # The rover goes grey rather than disappearing when the
                        # link drops -- see the note in arena_minimap_node.cpp.
                        "pose_timeout_s": 3.0,
                    }
                ],
            ),
        ]
    )
