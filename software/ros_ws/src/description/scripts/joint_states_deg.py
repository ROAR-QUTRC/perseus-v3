#!/usr/bin/env python3
"""Republish joint_states in degrees (and deg/s) as joint_states_deg, for debugging.

The optional `joints` parameter limits the output to those joints, in that order.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatesDeg(Node):
    def __init__(self):
        super().__init__("joint_states_deg")
        self._joints = list(self.declare_parameter("joints", [""]).value)
        self._joints = [j for j in self._joints if j]
        self._pub = self.create_publisher(JointState, "joint_states_deg", 10)
        self.create_subscription(JointState, "joint_states", self._on_states, 10)

    def _on_states(self, msg):
        index = range(len(msg.name))
        if self._joints:
            index = [msg.name.index(j) for j in self._joints if j in msg.name]
            if not index:
                return
        out = JointState()
        out.header = msg.header
        out.name = [msg.name[i] for i in index]
        out.position = [math.degrees(msg.position[i]) for i in index]
        if len(msg.velocity) == len(msg.name):
            out.velocity = [math.degrees(msg.velocity[i]) for i in index]
        if len(msg.effort) == len(msg.name):
            out.effort = [msg.effort[i] for i in index]
        self._pub.publish(out)


def main():
    rclpy.init()
    try:
        rclpy.spin(JointStatesDeg())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
