#!/usr/bin/env python3
"""Launch the terrain costmap, the global planner, and the nav2 stack that drives to a goal.

    /Laser_map -> global_traversability -> /costmap -> planner_server   -> /plan
                                                   -> smoother_server  -> (smoothed path)
                                                   -> controller_server -> /cmd_vel
                                                   -> velocity_smoother -> /cmd_vel_nav_stamped

THIS LAUNCH FILE CAN MOVE THE ROVER. With use_control true (the default) an rviz "2D Goal
Pose" makes it drive. bt_navigator subscribes to /goal_pose directly, so the rviz button
works without the nav2 rviz panel.

Two ways to stop it:
  * The joystick. twist_mux gives it priority 100 against navigation's 10, so touching the
    stick takes the rover off nav2 within one message and holds it for the 0.5 s timeout.
  * The e-stop. That is the one to actually rely on.

Three levels, so the dangerous part is opt-out rather than unavoidable:

    use_planner:=false                  costmap only. Nothing can move. Use this while
                                        tuning the thresholds in config/navigation.yaml.
    use_control:=false                  costmap + planner. Computes and publishes paths on
                                        /plan, cannot drive. Use scripts/plan_probe.py.
    (defaults)                          the full stack. The rover drives.

localisation.launch.py must already be running -- it supplies /Laser_map, the odom frame,
and the odom -> base_footprint every costmap here needs. perseus.launch.py must be running
too, for twist_mux and the diff drive controller that turn /cmd_vel_nav_stamped into wheel
motion.

Configured entirely by config/navigation.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    config_file = os.path.join(
        get_package_share_directory("autonomy_bringup"), "config", "navigation.yaml"
    )
    use_sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}

    # Resolved here rather than used as conditions because they change the SHAPE of the
    # launch: the lifecycle manager has to be told exactly which nodes it is managing, and
    # naming one that was never launched leaves it waiting forever on a bond that never
    # forms, which brings the whole stack up as "failed" with nothing obviously wrong.
    use_planner = IfCondition(LaunchConfiguration("use_planner")).evaluate(context)
    use_control = IfCondition(LaunchConfiguration("use_control")).evaluate(context)
    # The driving half calls ComputePathToPose on every replan, so it cannot run without
    # the planner. Rather than fail obscurely at the first goal, pull the planner in.
    if use_control:
        use_planner = True

    # nav2's stock behaviour tree never calls SmoothPath, so pointing bt_navigator at our
    # own tree is what actually enables smoother_server. An absolute path into the share
    # directory, which is why it cannot live in navigation.yaml.
    bt_xml = os.path.join(
        get_package_share_directory("autonomy_bringup"),
        "behavior_trees",
        "navigate_to_pose_w_smoothing.xml",
    )

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
        )
    ]
    # Lifecycle bringup order. nav2's own convention: the servers that answer requests come
    # up before the ones that make them, so bt_navigator never asks a planner that is not
    # yet listening.
    managed = []

    if use_planner:
        nodes.append(nav2_node("nav2_planner", "planner_server", "planner_server"))
        managed.append("planner_server")

    if use_control:
        nodes += [
            nav2_node("nav2_controller", "controller_server", "controller_server"),
            nav2_node("nav2_smoother", "smoother_server", "smoother_server"),
            nav2_node("nav2_behaviors", "behavior_server", "behavior_server"),
            nav2_node(
                "nav2_bt_navigator",
                "bt_navigator",
                "bt_navigator",
                extra_params=[{"default_nav_to_pose_bt_xml": bt_xml}],
            ),
            nav2_node(
                "nav2_waypoint_follower", "waypoint_follower", "waypoint_follower"
            ),
            # THE ONE REMAP THAT CONNECTS NAV2 TO THIS ROVER. The smoother's output is
            # nav2's last word on velocity; twist_mux's navigation input is
            # cmd_vel_nav_stamped (perseus/config/twist_mux.yaml). Without this the stack
            # runs perfectly and the wheels never turn.
            nav2_node(
                "nav2_velocity_smoother",
                "velocity_smoother",
                "velocity_smoother",
                remappings=[("cmd_vel_smoothed", "/cmd_vel_nav_stamped")],
            ),
        ]
        # controller_server first: it is what everything else ultimately drives.
        managed = [
            "controller_server",
            "smoother_server",
            "planner_server",
            "behavior_server",
            "bt_navigator",
            "waypoint_follower",
            "velocity_smoother",
        ]

    if managed:
        nodes.append(
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                # node_names is set here rather than in the yaml because it has to match
                # what was actually launched, which the arguments above decide.
                parameters=[config_file, {"node_names": managed}, use_sim_time],
                output="screen",
            )
        )

    return nodes


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the simulation clock. Only set true when something publishes "
            "/clock.",
        ),
        DeclareLaunchArgument(
            "use_planner",
            default_value="true",
            description="Launch nav2's global planner against /costmap. false brings up "
            "the terrain costmap alone, for tuning it with nothing else running.",
        ),
        DeclareLaunchArgument(
            "use_control",
            default_value="true",
            description="Launch the half of nav2 that DRIVES THE ROVER: controller, "
            "behaviours, BT navigator, waypoint follower and velocity smoother. false "
            "leaves the planner publishing paths on /plan without moving anything.",
        ),
    ]

    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])
