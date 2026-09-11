#!/usr/bin/env python3
import math

import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from moveit_msgs.srv import ServoCommandType
from std_msgs.msg import Bool, Empty, Float64


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_node")

        # Adjustable configuration
        self.declare_parameter("max_linear_speed", 0.20)  # m/s
        self.declare_parameter("max_joint_speed", 0.75)  # rad/s
        self.declare_parameter("max_gripper_speed", 0.02)  # m/s per finger
        self.declare_parameter("deadzone", 0.08)
        self.declare_parameter("joy_timeout", 0.25)  # seconds
        self.declare_parameter("mode_switch_settle_time", 0.1)  # seconds
        self.declare_parameter("toggle_button", 2)  # Triangle
        self.declare_parameter("mode_toggle_button", 3)  # Square
        self.declare_parameter("command_frame", "plate")

        self.max_linear = self.get_parameter("max_linear_speed").value
        self.max_joint = self.get_parameter("max_joint_speed").value
        self.max_gripper = self.get_parameter("max_gripper_speed").value
        self.deadzone = self.get_parameter("deadzone").value
        self.joy_timeout = self.get_parameter("joy_timeout").value
        self.mode_switch_settle_time = self.get_parameter(
            "mode_switch_settle_time"
        ).value
        self.toggle_button = self.get_parameter("toggle_button").value
        self.mode_toggle_button = self.get_parameter("mode_toggle_button").value
        self.command_frame = self.get_parameter("command_frame").value

        # Command publishers
        self.twist_publisher = self.create_publisher(
            TwistStamped,
            "command/end_effector_twist",
            10,
        )

        self.shoulder_publisher = self.create_publisher(
            JointJog, "command/shoulder_joint_jog", 10
        )
        self.wrist_bend_publisher = self.create_publisher(
            Float64, "command/wrist_bend_velocity", 10
        )
        self.wrist_twist_publisher = self.create_publisher(
            Float64, "command/wrist_twist_velocity", 10
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

        self.servo_mode_client = self.create_client(
            ServoCommandType, "/servo_node/switch_command_type"
        )

        self.command = [0.0] * 3
        self.shoulder_velocity = 0.0
        self.wrist_bend_velocity = 0.0
        self.wrist_twist_velocity = 0.0
        self.gripper_velocity = 0.0

        self.requested_mode = "cartesian"
        self.active_mode = None
        self.mode_request = None
        self.mode_request_after_ns = None
        self.mode_ready_after_ns = None
        self.last_service_warning = None

        self.enabled = False
        self.toggle_was_pressed = False
        self.mode_toggle_was_pressed = False
        self.l3_was_pressed = False
        self.r3_was_pressed = False
        self.last_joy_time = None

        # Publish commands at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_command)
        self.mode_timer = self.create_timer(0.25, self.ensure_servo_mode)

        self.get_logger().info(
            "Teleoperation started: LOCKED, Cartesian mode requested"
        )
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
        self.command = [0.0] * 3
        self.shoulder_velocity = 0.0
        self.wrist_bend_velocity = 0.0
        self.wrist_twist_velocity = 0.0
        self.gripper_velocity = 0.0

    def ensure_servo_mode(self):
        if self.active_mode == self.requested_mode:
            return
        now = self.get_clock().now()
        if (
            self.mode_request_after_ns is not None
            and now.nanoseconds < self.mode_request_after_ns
        ):
            return
        if self.mode_request is not None and not self.mode_request.done():
            return
        if not self.servo_mode_client.service_is_ready():
            if (
                self.last_service_warning is None
                or (now - self.last_service_warning).nanoseconds > 5_000_000_000
            ):
                self.get_logger().warning(
                    "Waiting for MoveIt Servo command-mode service"
                )
                self.last_service_warning = now
            return

        requested_mode = self.requested_mode
        request = ServoCommandType.Request()
        request.command_type = (
            ServoCommandType.Request.TWIST
            if requested_mode == "cartesian"
            else ServoCommandType.Request.JOINT_JOG
        )
        self.mode_request_after_ns = None
        self.mode_request = self.servo_mode_client.call_async(request)
        self.mode_request.add_done_callback(
            lambda future: self.on_servo_mode_response(future, requested_mode)
        )

    def on_servo_mode_response(self, future, requested_mode):
        try:
            response = future.result()
        except Exception as error:  # ROS service failure
            self.get_logger().error(f"Failed to switch Servo mode: {error}")
            return

        if not response.success:
            self.get_logger().error("MoveIt Servo rejected the mode switch")
            return

        self.active_mode = requested_mode
        self.mode_ready_after_ns = self.get_clock().now().nanoseconds + int(
            self.mode_switch_settle_time * 1_000_000_000
        )
        self.stop_motion()
        self.get_logger().info(
            f"Control mode: {requested_mode.replace(chr(95), chr(32)).title()}"
        )

    def toggle_control_mode(self):
        self.requested_mode = (
            "shoulder_pivot" if self.requested_mode == "cartesian" else "cartesian"
        )
        self.active_mode = None
        self.mode_request_after_ns = self.get_clock().now().nanoseconds + int(
            self.mode_switch_settle_time * 1_000_000_000
        )
        self.mode_ready_after_ns = None
        self.stop_motion()
        self.get_logger().info(
            f"Requesting control mode: "
            f"{self.requested_mode.replace(chr(95), chr(32)).title()}"
        )
        self.ensure_servo_mode()

    def joy_callback(self, msg):
        required_buttons = max(13, self.toggle_button + 1, self.mode_toggle_button + 1)
        if len(msg.axes) < 8 or len(msg.buttons) < required_buttons:
            self.get_logger().error("Unexpected /joy controller mapping")

            if self.enabled:
                self.enabled = False
                self.publish_enabled_state()

            self.stop_motion()
            return

        self.last_joy_time = self.get_clock().now()

        toggle_pressed = bool(msg.buttons[self.toggle_button])
        mode_toggle_pressed = bool(msg.buttons[self.mode_toggle_button])

        if mode_toggle_pressed and not self.mode_toggle_was_pressed:
            self.toggle_control_mode()
        self.mode_toggle_was_pressed = mode_toggle_pressed

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

        # Bumpers divert their stick axis to independent wrist jogging.
        self.wrist_twist_velocity = left_x * self.max_joint if l1_held else 0.0
        self.wrist_bend_velocity = -right_y * self.max_joint if r1_held else 0.0

        left_lateral = 0.0 if l1_held else left_x
        right_lateral = 0.0 if r1_held else right_x
        lateral = (
            left_lateral if abs(left_lateral) >= abs(right_lateral) else right_lateral
        )

        self.command = [0.0] * 3
        self.shoulder_velocity = 0.0

        if self.requested_mode == "cartesian":
            self.command = [
                0.0 if r1_held else right_y * self.max_linear,
                lateral * self.max_linear,
                0.0 if l1_held else left_y * self.max_linear,
            ]
        else:
            self.shoulder_velocity = lateral * self.max_joint

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

        shoulder = JointJog()
        shoulder.header.stamp = now.to_msg()
        shoulder.header.frame_id = self.command_frame
        shoulder.joint_names = ["shoulder_pan"]
        shoulder.velocities = [self.shoulder_velocity]

        wrist_bend = Float64()
        wrist_bend.data = self.wrist_bend_velocity
        wrist_twist = Float64()
        wrist_twist.data = self.wrist_twist_velocity
        gripper = Float64()
        gripper.data = self.gripper_velocity

        mode_ready = (
            self.active_mode == self.requested_mode
            and self.mode_ready_after_ns is not None
            and now.nanoseconds >= self.mode_ready_after_ns
        )
        if not self.enabled:
            twist.twist.linear.x = 0.0
            twist.twist.linear.y = 0.0
            twist.twist.linear.z = 0.0
            shoulder.velocities = [0.0]
            wrist_bend.data = 0.0
            wrist_twist.data = 0.0
            gripper.data = 0.0

        if mode_ready:
            if self.active_mode == "cartesian":
                self.twist_publisher.publish(twist)
            else:
                self.shoulder_publisher.publish(shoulder)

        self.wrist_bend_publisher.publish(wrist_bend)
        self.wrist_twist_publisher.publish(wrist_twist)
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
