#!/usr/bin/env python3
"""Bridges command/wrist_bend_velocity and command/wrist_twist_velocity
(signed rad/s) to the wrist trajectory controller."""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

LIMITS = {"wrist_pitch": (-1.57, 1.57), "wrist_roll": (-6.28318, 6.28318)}


class WristVelocityBridge(Node):
    def __init__(self) -> None:
        super().__init__("wrist_velocity_bridge")
        self.declare_parameter("max_speed", 1.0)  # rad/s
        self.declare_parameter("command_timeout", 0.25)
        self.declare_parameter("joint_state_topic", "joint_states")
        self.declare_parameter("bend_command_topic", "command/wrist_bend_velocity")
        self.declare_parameter("twist_command_topic", "command/wrist_twist_velocity")
        self.declare_parameter("trajectory_topic", "wrist_controller/joint_trajectory")

        self.velocity = {"wrist_pitch": 0.0, "wrist_roll": 0.0}
        self.target = None  # seeded from joint states
        self.last_cmd_time = self.get_clock().now()

        self.create_subscription(
            JointState,
            self.get_parameter("joint_state_topic").value,
            self.on_joint_states,
            10,
        )
        self.create_subscription(
            Float64,
            self.get_parameter("bend_command_topic").value,
            lambda m: self.on_cmd("wrist_pitch", m),
            10,
        )
        self.create_subscription(
            Float64,
            self.get_parameter("twist_command_topic").value,
            lambda m: self.on_cmd("wrist_roll", m),
            10,
        )
        self.pub = self.create_publisher(
            JointTrajectory, self.get_parameter("trajectory_topic").value, 10
        )
        self.period = 0.02
        self.create_timer(self.period, self.tick)

    def on_joint_states(self, msg: JointState) -> None:
        if self.target is not None:
            return
        try:
            self.target = {j: msg.position[msg.name.index(j)] for j in self.velocity}
        except ValueError:
            pass

    def on_cmd(self, joint: str, msg: Float64) -> None:
        limit = self.get_parameter("max_speed").value
        self.velocity[joint] = max(-limit, min(limit, msg.data))
        self.last_cmd_time = self.get_clock().now()

    def tick(self) -> None:
        timeout = Duration(seconds=self.get_parameter("command_timeout").value)
        if self.get_clock().now() - self.last_cmd_time > timeout:
            self.velocity = dict.fromkeys(self.velocity, 0.0)
        if self.target is None or all(v == 0.0 for v in self.velocity.values()):
            return
        for joint, vel in self.velocity.items():
            lo, hi = LIMITS[joint]
            self.target[joint] = max(
                lo, min(hi, self.target[joint] + vel * self.period)
            )
        point = JointTrajectoryPoint(
            positions=[self.target["wrist_pitch"], self.target["wrist_roll"]],
            velocities=[0.0, 0.0],
            time_from_start=Duration(seconds=2 * self.period).to_msg(),
        )
        self.pub.publish(
            JointTrajectory(joint_names=["wrist_pitch", "wrist_roll"], points=[point])
        )


def main() -> None:
    rclpy.init()
    node = WristVelocityBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
