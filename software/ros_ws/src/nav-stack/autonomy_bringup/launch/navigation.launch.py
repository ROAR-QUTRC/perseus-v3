#!/usr/bin/env python3
"""Launch the terrain costmap and the global planner that plans against it.

    /Laser_map  ->  global_traversability  ->  /costmap  ->  planner_server  ->  /plan

global_traversability turns BIEVR-LIO's accumulated map cloud into a terrain-aware occupancy
costmap, in place of a global costmap sourced only from a 2D SLAM map. planner_server is
nav2's global planner, pointed at that costmap through nav2's static layer.

Deliberately NOT here: bt_navigator, controller_server, behavior_server. Nothing this file
launches can move the rover -- it computes paths and publishes them on /plan, and a bad one
is something to look at in rviz rather than something the rover does. Set use_planner false
to bring up the costmap alone, which is what you want while tuning the thresholds in
config/navigation.yaml.

localisation.launch.py must already be running. It supplies /Laser_map, the odom frame, and
the odom -> base_footprint that the planner's costmap needs to locate the robot: the
flattening broadcaster that produces it is launched there, next to the EKF whose
odom -> base_link it is derived from.

planner_server answers an ACTION, not a service, so a path is requested with:

  ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
    "{goal: {header: {frame_id: odom}, pose: {position: {x: 3.0, y: 0.0}, \
      orientation: {w: 1.0}}}, use_start: false}"

or with scripts/plan_probe.py, which drives it from rviz's "2D Goal Pose" button and reports
path length, detour ratio and planning time.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    declare_use_planner = DeclareLaunchArgument(
        "use_planner",
        default_value="true",
        description="Launch nav2's global planner against /costmap. false brings up the "
        "terrain costmap alone, for tuning it without a planner running.",
    )

    use_sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}
    use_planner = IfCondition(LaunchConfiguration("use_planner"))

    global_traversability_node = Node(
        package="global_traversability",
        executable="global_traversability",
        name="global_traversability",
        parameters=[config_file, use_sim_time],
        output="screen",
    )

    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        parameters=[config_file, use_sim_time],
        output="screen",
        condition=use_planner,
    )

    # planner_server comes up unconfigured and stays that way until something walks it
    # through configure -> activate. autostart in navigation.yaml makes that automatic;
    # without this node the server launches, logs nothing wrong, and never answers an action.
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        parameters=[config_file, use_sim_time],
        output="screen",
        condition=use_planner,
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_use_planner,
            global_traversability_node,
            planner_server_node,
            lifecycle_manager_node,
        ]
    )
