#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Empty, Float64


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_node")

        # Adjustable configuration
        self.declare_parameter("max_linear_speed", 0.20)  # m/s
        self.declare_parameter("max_angular_speed", 0.75)  # rad/s
        self.declare_parameter("max_gripper_speed", 1.0)  # rad/s
        self.declare_parameter("deadzone", 0.08)
        self.declare_parameter("joy_timeout", 0.25)  # seconds
        self.declare_parameter("toggle_button", 2)  # Triangle
        self.declare_parameter("command_frame", "base_link")

        self.max_linear = self.get_parameter("max_linear_speed").value
        self.max_angular = self.get_parameter("max_angular_speed").value
        self.max_gripper = self.get_parameter("max_gripper_speed").value
        self.deadzone = self.get_parameter("deadzone").value
        self.joy_timeout = self.get_parameter("joy_timeout").value
        self.toggle_button = self.get_parameter("toggle_button").value
        self.command_frame = self.get_parameter("command_frame").value

        # Command publishers
        self.twist_publisher = self.create_publisher(
            TwistStamped,
            "command/end_effector_twist",
            10,
        )

        self.gripper_publisher = self.create_publisher(
            Float64,
            "command/gripper_velocity",
            10,
        )

        self.reset_roll_publisher = self.create_publisher(
            Empty,
            "command/reset_roll",
            10,
        )

        self.reset_pitch_yaw_publisher = self.create_publisher(
            Empty,
            "command/reset_pitch_yaw",
            10,
        )

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.enabled_publisher = self.create_publisher(
            Bool,
            "teleop/enabled",
            status_qos,
        )

        # Controller input
        self.joy_subscription = self.create_subscription(
            Joy,
            "joy",
            self.joy_callback,
            10,
        )

        self.command = [0.0] * 6
        self.gripper_velocity = 0.0

        self.enabled = False
        self.toggle_was_pressed = False
        self.l3_was_pressed = False
        self.r3_was_pressed = False
        self.last_joy_time = None

        # Publish commands at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_command)

        self.get_logger().info("Teleoperation started: LOCKED")
        self.publish_enabled_state()

    def publish_enabled_state(self):
        state = Bool()
        state.data = self.enabled
        self.enabled_publisher.publish(state)

    def apply_deadzone(self, value):
        """Remove stick drift and rescale the remaining range."""
        if abs(value) <= self.deadzone:
            return 0.0

        scaled = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        return math.copysign(scaled, value)

    @staticmethod
    def trigger_amount(value):
        """Convert trigger range from [+1, -1] to [0, 1]."""
        value = max(-1.0, min(1.0, value))
        return (1.0 - value) / 2.0

    def stop_motion(self):
        self.command = [0.0] * 6
        self.gripper_velocity = 0.0

    def joy_callback(self, msg):
        if len(msg.axes) < 8 or len(msg.buttons) < 13:
            self.get_logger().error("Unexpected /joy controller mapping")

            if self.enabled:
                self.enabled = False
                self.publish_enabled_state()

            self.stop_motion()
            return

        self.last_joy_time = self.get_clock().now()

        toggle_pressed = bool(msg.buttons[self.toggle_button])

        # Toggle only once per Triangle press
        if toggle_pressed and not self.toggle_was_pressed:
            if self.enabled:
                self.enabled = False
                self.stop_motion()
                self.publish_enabled_state()
                self.get_logger().info("Teleoperation LOCKED")
            else:
                sticks_neutral = all(
                    abs(msg.axes[index]) <= self.deadzone for index in (0, 1, 3, 4)
                )

                triggers_released = not msg.buttons[6] and not msg.buttons[7]

                if sticks_neutral and triggers_released:
                    self.enabled = True
                    self.publish_enabled_state()
                    self.get_logger().info("Teleoperation UNLOCKED")
                else:
                    self.get_logger().warning(
                        "Cannot unlock: release sticks and triggers"
                    )

        self.toggle_was_pressed = toggle_pressed

        l1_held = bool(msg.buttons[4])
        r1_held = bool(msg.buttons[5])
        l3_pressed = bool(msg.buttons[11])
        r3_pressed = bool(msg.buttons[12])

        # Publish reset requests once per stick click
        if self.enabled:
            if l1_held and l3_pressed and not self.l3_was_pressed:
                self.reset_roll_publisher.publish(Empty())
                self.get_logger().info("Roll reset requested")

            if r1_held and r3_pressed and not self.r3_was_pressed:
                self.reset_pitch_yaw_publisher.publish(Empty())
                self.get_logger().info("Pitch/yaw reset requested")

        self.l3_was_pressed = l3_pressed
        self.r3_was_pressed = r3_pressed

        if not self.enabled:
            self.stop_motion()
            return

        left_x = self.apply_deadzone(msg.axes[0])
        left_y = self.apply_deadzone(msg.axes[1])
        right_x = self.apply_deadzone(msg.axes[3])
        right_y = self.apply_deadzone(msg.axes[4])

        # Start with zero motion
        linear_x = 0.0
        linear_y = 0.0
        linear_z = 0.0

        roll = 0.0
        pitch = 0.0
        yaw = 0.0

        # Left-stick mode
        if l1_held:
            # Horizontal movement becomes roll.
            roll = left_x
            left_lateral = 0.0
        else:
            # Normal vertical-plane translation.
            left_lateral = left_x
            linear_z = left_y

        # Right-stick mode
        if r1_held:
            # Both right-stick directions remain active:
            # horizontal -> yaw
            # vertical   -> pitch
            yaw = right_x
            pitch = -right_y
            right_lateral = 0.0
        else:
            # Normal horizontal-plane translation.
            right_lateral = right_x
            linear_x = right_y

        # Select the stronger lateral command without adding them.
        if abs(left_lateral) >= abs(right_lateral):
            linear_y = left_lateral
        else:
            linear_y = right_lateral

        self.command = [
            linear_x * self.max_linear,
            linear_y * self.max_linear,
            linear_z * self.max_linear,
            roll * self.max_angular,
            pitch * self.max_angular,
            yaw * self.max_angular,
        ]

        close_amount = self.trigger_amount(msg.axes[2])
        open_amount = self.trigger_amount(msg.axes[5])

        # Positive opens; negative closes
        self.gripper_velocity = (open_amount - close_amount) * self.max_gripper

    def publish_command(self):
        now = self.get_clock().now()

        timed_out = (
            self.last_joy_time is None
            or (now - self.last_joy_time).nanoseconds / 1e9 > self.joy_timeout
        )

        if timed_out:
            if self.enabled:
                self.enabled = False
                self.publish_enabled_state()

            self.stop_motion()

        twist = TwistStamped()
        twist.header.stamp = now.to_msg()
        twist.header.frame_id = self.command_frame

        twist.twist.linear.x = self.command[0]
        twist.twist.linear.y = self.command[1]
        twist.twist.linear.z = self.command[2]

        twist.twist.angular.x = self.command[3]
        twist.twist.angular.y = self.command[4]
        twist.twist.angular.z = self.command[5]

        gripper = Float64()
        gripper.data = self.gripper_velocity

        self.twist_publisher.publish(twist)
        self.gripper_publisher.publish(gripper)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

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
