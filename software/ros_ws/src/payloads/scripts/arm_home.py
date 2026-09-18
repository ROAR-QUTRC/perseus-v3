#!/usr/bin/env python3
"""Drive every arm joint to initial_positions.yaml:
ros2 service call /arm_home/move std_srvs/srv/Trigger"""

import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

CONTROLLERS = {
    "servo_controller": ["shoulder_pan", "shoulder_tilt", "elbow"],
    "wrist_controller": ["wrist_pitch", "wrist_roll"],
    "gripper_controller": ["left_finger_joint"],
}


class ArmHome(Node):
    def __init__(self) -> None:
        super().__init__("arm_home")
        self.declare_parameter(
            "speed", 0.5
        )  # rad/s; the farthest joint sets the duration
        path = os.path.join(
            get_package_share_directory("payloads"), "config", "initial_positions.yaml"
        )
        with open(path) as f:
            self.target = yaml.safe_load(f)["initial_positions"]
        self.state = {}
        group = ReentrantCallbackGroup()
        self.create_subscription(
            JointState,
            "/joint_states",
            lambda m: self.state.update(zip(m.name, m.position)),
            10,
            callback_group=group,
        )
        self.pubs = {
            c: self.create_publisher(JointTrajectory, f"/{c}/joint_trajectory", 10)
            for c in CONTROLLERS
        }
        self.pause = self.create_client(
            SetBool, "/servo_node/pause_servo", callback_group=group
        )
        self.create_service(Trigger, "~/move", self.move, callback_group=group)

    def set_servo_paused(self, paused: bool) -> None:
        if not self.pause.wait_for_service(timeout_sec=0.5):
            return
        future = self.pause.call_async(SetBool.Request(data=paused))
        while not future.done():
            time.sleep(0.02)

    def move(self, _request, response):
        missing = [j for j in self.target if j not in self.state]
        if missing:
            response.message = f"no joint state for {missing}"
            return response
        speed = self.get_parameter("speed").value
        duration = max(
            2.0, max(abs(self.target[j] - self.state[j]) for j in self.target) / speed
        )
        # Servo republishes its own hold trajectory every cycle; pause it for the move.
        self.set_servo_paused(True)
        for controller, joints in CONTROLLERS.items():
            point = JointTrajectoryPoint(
                positions=[self.target[j] for j in joints],
                time_from_start=Duration(seconds=duration).to_msg(),
            )
            self.pubs[controller].publish(
                JointTrajectory(joint_names=joints, points=[point])
            )
        time.sleep(duration + 0.5)
        self.set_servo_paused(False)
        response.success = True
        response.message = f"moved to initial positions in {duration:.1f} s"
        return response


def main() -> None:
    rclpy.init()
    node = ArmHome()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
