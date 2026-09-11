#!/usr/bin/env python3
"""Bridges command/gripper_velocity (signed velocity) to the gripper's
trajectory controller by integrating it into a position target."""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GripperVelocityBridge(Node):
    def __init__(self) -> None:
        super().__init__("gripper_velocity_bridge")
        self.declare_parameter("max_speed", 0.02)  # m/s per finger
        self.declare_parameter("joint_upper", 0.015)  # m, closed is 0.0
        self.declare_parameter("command_timeout", 0.25)
        self.declare_parameter("command_topic", "command/gripper_velocity")
        self.declare_parameter(
            "trajectory_topic", "gripper_controller/joint_trajectory"
        )

        self.velocity = 0.0
        self.target = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.create_subscription(
            Float64,
            self.get_parameter("command_topic").value,
            self.on_cmd,
            10,
        )
        self.pub = self.create_publisher(
            JointTrajectory, self.get_parameter("trajectory_topic").value, 10
        )
        self.period = 0.02
        self.create_timer(self.period, self.tick)

    def on_cmd(self, msg: Float64) -> None:
        limit = self.get_parameter("max_speed").value
        self.velocity = max(-limit, min(limit, msg.data))
        self.last_cmd_time = self.get_clock().now()

    def tick(self) -> None:
        timeout = Duration(seconds=self.get_parameter("command_timeout").value)
        if self.get_clock().now() - self.last_cmd_time > timeout:
            self.velocity = 0.0
        if self.velocity == 0.0:
            return
        upper = self.get_parameter("joint_upper").value
        self.target = max(0.0, min(upper, self.target + self.velocity * self.period))
        point = JointTrajectoryPoint(
            positions=[self.target],
            velocities=[0.0],
            time_from_start=Duration(seconds=2 * self.period).to_msg(),
        )
        self.pub.publish(
            JointTrajectory(joint_names=["left_finger_joint"], points=[point])
        )


def main() -> None:
    rclpy.init()
    node = GripperVelocityBridge()
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
