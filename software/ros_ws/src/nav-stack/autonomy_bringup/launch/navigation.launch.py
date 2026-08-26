#!/usr/bin/env python3
"""Launch the nodes nav2's global costmap depends on, but that nav2 itself does not provide.

global_traversability turns FAST-LIO's accumulated map cloud (/Laser_map) into a
terrain-aware occupancy costmap, in place of a global costmap sourced only from a 2D SLAM
map. Point nav2's global costmap static layer at its costmap topic once nav2 itself is
brought up elsewhere. It is parameterised by config/navigation.yaml.

localisation.launch.py must already be running to supply /Laser_map, and also the
odom -> base_footprint that nav2's costmaps and controller need: the flattening
broadcaster that produces it is launched there, next to the EKF whose odom -> base_link
it is derived from.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("autonomy_bringup"), "config", "navigation.yaml"
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use the simulation clock. Only set true when something publishes "
        "/clock.",
    )

    global_traversability_node = Node(
        package="global_traversability",
        executable="global_traversability",
        name="global_traversability",
        parameters=[config_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            global_traversability_node,
        ]
    )
