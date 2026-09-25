#!/usr/bin/env python3
"""Launch the terrain costmaps and the full nav2 stack that drives to a goal.

    /livox/lidar -> global_traversability -> /costmap ---------------> planner_server   -> /plan
                                                       |                smoother_server
    /livox/lidar -> local_traversability                -> controller_server -> /cmd_vel
                    -> /local_costmap_terrain --------/                 velocity_smoother
                                                                          -> /cmd_vel_nav_stamped

TWO TERRAIN COSTMAPS, NOT ONE, AND THEY ARE NOT INTERCHANGEABLE. Both classify the raw Livox
scan with the same height-difference test. global_traversability REMEMBERS every cell it has
seen, re-deciding a cell only when the lidar looks at it again, so it covers the whole arena
and is what the planner plans on. local_traversability forgets after 1 s over an 8 m window,
so it is what the controller reacts to. Both land in the controller's local costmap, the
local one layered on top with use_maximum -- see local_costmap in config/navigation.yaml.

THIS LAUNCH FILE MOVES THE ROVER. An rviz "2D Goal Pose" makes it drive: bt_navigator
subscribes to /goal_pose directly, so the button works without the nav2 rviz panel.

Two ways to stop it:
  * The joystick. twist_mux gives it priority 100 against navigation's 10, so touching the
    stick takes the rover off nav2 within one message.
  * The e-stop. That is the one to actually rely on.

Prerequisites:
  localisation.launch.py -- the odom frame, odom -> base_footprint and odom -> the lidar.
  perseus.launch.py      -- twist_mux and the diff drive controller that turn
                            /cmd_vel_nav_stamped into wheel motion.

Configured entirely by config/navigation.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("autonomy_bringup")
    config_file = os.path.join(share, "config", "navigation.yaml")
    # nav2's stock behaviour tree never calls SmoothPath, so pointing bt_navigator at our
    # own tree is what actually enables smoother_server. An absolute path into the share
    # directory, which is why it cannot live in navigation.yaml.
    bt_xml = os.path.join(share, "behavior_trees", "navigate_to_pose_w_smoothing.xml")
    # mission_bt_server's own small tree: request a safe zone waypoint from arena_server,
    # then hand it to bt_navigator's own /navigate_to_pose action - i.e. the tree above,
    # unmodified. Same reason this path cannot live in navigation.yaml either.
    mission_bt_xml = os.path.join(share, "behavior_trees", "go_to_zone_waypoint.xml")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use the simulation clock. Only set true when something publishes "
        "/clock.",
    )
    use_sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}

    def nav2_node(package, executable, name, remappings=None, extra_params=None):
        return Node(
            package=package,
            executable=executable,
            name=name,
            parameters=[config_file, *(extra_params or []), use_sim_time],
            remappings=remappings or [],
            output="screen",
        )

    nodes = [
        Node(
            package="global_traversability",
            executable="global_traversability",
            name="global_traversability",
            parameters=[config_file, use_sim_time],
            output="screen",
        ),
        # The reactive half. Not a lifecycle node and not in lifecycle_manager's
        # node_names below: it is a plain publisher, so it comes up with the process and
        # the local costmap picks its grid up whenever it appears. It needs TF
        # (odom -> livox_frame and odom -> base_footprint) and the raw Livox scan, both
        # of which come from localisation.launch.py and the sensor drivers -- without
        # them it logs a throttled "Dropping scan" and publishes nothing, which is the
        # intended failure: nav2 then runs on the global terrain map alone, exactly as
        # it did before this node existed.
        Node(
            package="local_traversability",
            executable="local_traversability",
            name="local_traversability",
            parameters=[config_file, use_sim_time],
            output="screen",
        ),
        nav2_node("nav2_controller", "controller_server", "controller_server"),
        nav2_node("nav2_smoother", "smoother_server", "smoother_server"),
        nav2_node("nav2_planner", "planner_server", "planner_server"),
        nav2_node("nav2_behaviors", "behavior_server", "behavior_server"),
        nav2_node(
            "nav2_bt_navigator",
            "bt_navigator",
            "bt_navigator",
            extra_params=[{"default_nav_to_pose_bt_xml": bt_xml}],
        ),
        nav2_node("nav2_waypoint_follower", "waypoint_follower", "waypoint_follower"),
        # Behind the RViz mission panel's two buttons. Brought up here rather than with
        # localisation (where arena_server lives): it needs bt_navigator's
        # /navigate_to_pose action, which only exists once this file's nodes are active.
        Node(
            package="mission_bt_server",
            executable="mission_bt_server",
            name="mission_bt_server",
            parameters=[{"bt_xml_path": mission_bt_xml}, use_sim_time],
            output="screen",
        ),
        # THE ONE REMAP THAT CONNECTS NAV2 TO THIS ROVER. The smoother's output is nav2's
        # last word on velocity; twist_mux's navigation input is cmd_vel_nav_stamped
        # (perseus/config/twist_mux.yaml). Without this the stack runs perfectly and the
        # wheels never turn.
        nav2_node(
            "nav2_velocity_smoother",
            "velocity_smoother",
            "velocity_smoother",
            remappings=[("cmd_vel_smoothed", "/cmd_vel_nav_stamped")],
        ),
        # Every nav2 node above is a lifecycle node: launched, it sits unconfigured and
        # answers nothing. This walks them to active on startup. The order it does that in
        # is node_names in navigation.yaml, and it must list exactly the nodes launched
        # here -- naming one that is missing hangs bringup on a bond that never forms.
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            parameters=[config_file, use_sim_time],
            output="screen",
        ),
    ]

    return LaunchDescription([declare_use_sim_time] + nodes)
