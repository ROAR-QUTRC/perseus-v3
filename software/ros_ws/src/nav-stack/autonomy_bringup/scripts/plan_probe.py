#!/usr/bin/env python3
"""Turn rviz's "2D Goal Pose" button into a global-planner test harness.

planner_server answers the ComputePathToPose action. Rviz's goal button only publishes
/goal_pose and expects bt_navigator to be listening, which it is not when the planner is
running alone -- so clicking a goal does nothing on its own. This bridges the two: every
/goal_pose becomes one ComputePathToPose call, and the result is reported as numbers rather
than left to be judged by eye off the /plan display.

    python3 plan_probe.py                     # goals from rviz
    python3 plan_probe.py --goal 3.0 1.5      # one goal, then exit

What to look for, in rough order of what goes wrong first:
  * "planner_server not available"  -- the lifecycle manager did not activate it, or the
                                       daemon is stale. Check `ros2 lifecycle get
                                       /planner_server` says active.
  * error_code 202 (TF_ERROR)       -- no odom -> base_footprint. localisation.launch.py is
                                       not running, or global_frame does not match the LIO's
                                       map.frame.
  * error_code 203/204 OUTSIDE_MAP  -- the costmap never arrived (QoS: the static layer needs
                                       map_subscribe_transient_local true), or the goal is
                                       genuinely off the mapped area.
  * error_code 205/206 OCCUPIED     -- the rover or the goal is sitting in a cell the terrain
                                       analysis calls lethal. With false obstacles on flat
                                       ground this is the one you will hit.
  * error_code 208 NO_VALID_PATH    -- reachable in principle, walled off in practice. Look at
                                       /costmap in rviz before blaming the planner.
  * a path that plans but hugs obstacles -- trinary_costmap is probably still true, so the
                                       inflation gradient was flattened to free space.
"""

import argparse
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose

# Names for the result error_code field, from the action definition. Reported rather than
# printed raw because "206" is not a diagnosis and "GOAL_OCCUPIED" is.
ERROR_NAMES = {
    0: "NONE",
    200: "UNKNOWN",
    201: "INVALID_PLANNER",
    202: "TF_ERROR",
    203: "START_OUTSIDE_MAP",
    204: "GOAL_OUTSIDE_MAP",
    205: "START_OCCUPIED",
    206: "GOAL_OCCUPIED",
    207: "TIMEOUT",
    208: "NO_VALID_PATH",
}


def path_length_m(path):
    """Summed straight-line distance between consecutive poses, in metres."""
    poses = path.poses
    total = 0.0
    for previous, current in zip(poses, poses[1:]):
        total += math.hypot(
            current.pose.position.x - previous.pose.position.x,
            current.pose.position.y - previous.pose.position.y,
        )
    return total


class PlanProbe(Node):
    def __init__(self, frame_id, once):
        super().__init__("plan_probe")
        self._frame_id = frame_id
        self._once = once
        self._client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self._pending = False

        if not once:
            self.create_subscription(PoseStamped, "/goal_pose", self._on_goal_pose, 1)
            self.get_logger().info(
                'Ready. Click "2D Goal Pose" in rviz to request a path.'
            )

    def _on_goal_pose(self, msg):
        # Rviz stamps the goal with whatever its Fixed Frame is. Overriding it keeps a
        # mismatched rviz setting from being reported as a planner TF failure.
        if msg.header.frame_id != self._frame_id:
            self.get_logger().warning(
                f"goal arrived in frame '{msg.header.frame_id}', treating it as "
                f"'{self._frame_id}' -- set rviz's Fixed Frame to {self._frame_id} to "
                "silence this"
            )
            msg.header.frame_id = self._frame_id
        self.send(msg)

    def send(self, goal_pose):
        if self._pending:
            self.get_logger().warning("a plan is already in flight, ignoring this goal")
            return
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "planner_server not available on /compute_path_to_pose after 5 s"
            )
            if self._once:
                rclpy.shutdown()
            return

        goal = ComputePathToPose.Goal()
        goal.goal = goal_pose
        # False means "plan from wherever the robot actually is", which is what makes this a
        # test of the live TF and costmap rather than of two arbitrary points.
        goal.use_start = False

        self._pending = True
        self.get_logger().info(
            f"requesting path to ({goal_pose.pose.position.x:.2f}, "
            f"{goal_pose.pose.position.y:.2f}) in {goal_pose.header.frame_id}"
        )
        self._client.send_goal_async(goal).add_done_callback(self._on_accepted)

    def _on_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("goal rejected by planner_server")
            self._finish()
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        code = result.error_code
        name = ERROR_NAMES.get(code, f"unrecognised({code})")

        if code != 0 or not result.path.poses:
            detail = f" -- {result.error_msg}" if result.error_msg else ""
            self.get_logger().error(f"no path: {name}{detail}")
            self._finish()
            return

        planning_ms = (
            result.planning_time.sec * 1000.0 + result.planning_time.nanosec / 1e6
        )
        poses = result.path.poses
        straight = math.hypot(
            poses[-1].pose.position.x - poses[0].pose.position.x,
            poses[-1].pose.position.y - poses[0].pose.position.y,
        )
        length = path_length_m(result.path)
        # Ratio of path length to straight-line distance. 1.0 is a straight shot; much above
        # about 1.5 on open ground means the planner is routing around something, which is
        # either a real obstacle or a false one worth looking at in /costmap.
        detour = length / straight if straight > 1e-6 else float("nan")

        self.get_logger().info(
            f"path ok: {len(poses)} poses, {length:.2f} m "
            f"(straight {straight:.2f} m, detour x{detour:.2f}), "
            f"planned in {planning_ms:.1f} ms"
        )
        self._finish()

    def _finish(self):
        self._pending = False
        if self._once:
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--goal",
        nargs=2,
        type=float,
        metavar=("X", "Y"),
        help="send one goal at these coordinates and exit, instead of listening to rviz",
    )
    parser.add_argument(
        "--frame",
        default="odom",
        help="frame the goal is expressed in (default: odom, the LIO's world frame -- there "
        "is no map frame on this robot)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = PlanProbe(args.frame, once=args.goal is not None)

    if args.goal is not None:
        pose = PoseStamped()
        pose.header.frame_id = args.frame
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = args.goal
        pose.pose.orientation.w = 1.0
        node.send(pose)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
