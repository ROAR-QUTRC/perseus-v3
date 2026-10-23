#!/usr/bin/env python3
"""Reverse a short, fixed distance WITHOUT consulting the costmap.

Why this exists. When every DWB trajectory hits an obstacle the rover's footprint is already
on a lethal cell. nav2's BackUp and Spin collision-check the footprint against the local
costmap before moving, and that check includes the pose the rover is standing in, so both
abort within a tick with "Collision Ahead" and nothing in the recovery list can move it.
This is the one recovery that can start from there.

How the behaviour tree reaches it. bt_navigator can only run BT nodes from plugin libraries
it was built with, and it has no generic "call a service" node. ClearEntireCostmap, though,
calls any nav2_msgs/srv/ClearEntireCostmap by name, so this node serves that type under
`escape/reverse` and the tree calls it with a ClearEntireCostmap node. The name is a lie
on the BT side; the service type is the only part that matters.

    <ClearEntireCostmap name="EscapeReverse" service_name="escape/reverse"/>

It is BLIND. The terrain grid has no coverage behind the rover, so nothing here knows what
is back there. Keep the distance short and the speed low, and run it only after the checked
BackUp has failed.

Output goes to cmd_vel_nav_stamped, twist_mux's navigation input (priority 10), so the
joystick, keyboard and web UI all still take over instantly. velocity_smoother publishes on
the same topic; it is idle while the controller is aborted, which is when this runs.

Parameters (ros2 param set /escape_reverse <name> <value>):
  distance_m   0.3    How far to reverse.
  speed_mps    0.15   Reverse speed. Keep above the ESCs' ~0.05 m/s stall.
  topic        cmd_vel_nav_stamped
  frame_id     base_link
"""

import threading
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.node import Node

PUBLISH_PERIOD_S = 0.05
# Zeros after the move so the diff drive controller stops, not coasts on its last command.
STOP_HOLD_S = 0.3


class EscapeReverse(Node):
    def __init__(self):
        super().__init__("escape_reverse")
        self._distance_m = self.declare_parameter("distance_m", 0.3).value
        self._speed_mps = self.declare_parameter("speed_mps", 0.15).value
        topic = self.declare_parameter("topic", "cmd_vel_nav_stamped").value
        self._frame_id = self.declare_parameter("frame_id", "base_link").value
        self._pub = self.create_publisher(TwistStamped, topic, 10)
        self._srv = self.create_service(
            ClearEntireCostmap, "escape/reverse", self._on_request
        )
        # A second request while one is running would stack two reverses.
        self._busy = threading.Lock()
        self.get_logger().info(
            f"ready: escape/reverse -> {self._distance_m} m at {self._speed_mps} m/s on {topic}"
        )

    def _send(self, vx):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.twist.linear.x = vx
        self._pub.publish(msg)

    def _on_request(self, _request, response):
        # Answer at once and reverse on a thread. The BT's service node gives up after
        # about bt_loop_duration / default_server_timeout (tens of ms in navigation.yaml,
        # which are milliseconds), so a reply that waits out the 2 s reverse is reported
        # as "Node timed out" and the tree moves on to the next recovery while the rover
        # is still backing up. The tree waits out the reverse itself with a Wait node.
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn("escape already running, ignoring request")
            return response
        threading.Thread(target=self._reverse, daemon=True).start()
        return response

    def _reverse(self):
        # Holds _busy from _on_request until the move is finished.
        try:
            distance = abs(float(self._distance_m))
            speed = abs(float(self._speed_mps))
            if speed <= 0.0 or distance <= 0.0:
                self.get_logger().error("distance_m and speed_mps must be positive")
                return
            duration = distance / speed
            self.get_logger().warn(
                f"escaping: reversing {distance:.2f} m at {speed:.2f} m/s, no collision check"
            )
            end = time.monotonic() + duration
            while time.monotonic() < end:
                self._send(-speed)
                time.sleep(PUBLISH_PERIOD_S)
            stop_end = time.monotonic() + STOP_HOLD_S
            while time.monotonic() < stop_end:
                self._send(0.0)
                time.sleep(PUBLISH_PERIOD_S)
        finally:
            self._busy.release()


def main():
    rclpy.init()
    node = EscapeReverse()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
