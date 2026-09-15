# Copyright 2026 Omar Kassab
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


"""Subscribe to mobility status and log dry-run decisions only."""

import time

from interfaces.msg import MobilityStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from slip_response.recovery_logic import Decision, ResponseConfig, SlipResponseLogic


class SlipResponseNode(Node):
    """Read-only prototype: no publishers, action clients, or motor control."""

    def __init__(self):
        """Load parameters and create the subscription and watchdog timer."""
        super().__init__('slip_response')
        topic = self.declare_parameter(
            'status_topic', '/watchdog/mobility_status'
        ).value
        defaults = ResponseConfig()
        values = {}
        for name, default in vars(defaults).items():
            values[name] = self.declare_parameter(name, default).value
        self._logic = SlipResponseLogic(
            ResponseConfig(**values), started_at_s=time.monotonic()
        )
        self._last_decision = None
        self._subscription = self.create_subscription(
            MobilityStatus, topic, self._on_status, QoSProfile(depth=1)
        )
        self._timer = self.create_timer(0.1, self._on_timer)
        self.get_logger().info(
            f'[DRY RUN] Listening to {topic}. '
            'Decisions are logs only: no velocity commands or Nav2 actions.'
        )

    def _on_status(self, message):
        # Receipt-time freshness detects a silent watchdog. Production control
        # also needs source timestamp, odometry health, and frame validation.
        self._logic.receive(
            message.is_evaluating, message.is_slipping, time.monotonic()
        )

    def _on_timer(self):
        decision = self._logic.decide(time.monotonic())
        if decision == self._last_decision:
            return
        self._last_decision = decision
        explanations = {
            Decision.WAITING_FOR_DATA: 'waiting for the first status message',
            Decision.IDLE: 'watchdog is not evaluating; this is not recovery proof',
            Decision.NORMAL: 'fresh evaluated assessment does not report slip',
            Decision.SLOW_DOWN: 'would request reduced speed; no command sent',
            Decision.STOP: 'would request a controlled stop; no command sent',
            Decision.RECOVERY_REQUIRED: (
                'would request autonomous recovery/replanning; no action sent'
            ),
            Decision.SENSOR_FAULT: (
                'status absent or stale; movement health is unknown'
            ),
        }
        text = f'[DRY RUN] {decision.value}: {explanations[decision]}'
        if decision in (Decision.STOP, Decision.SENSOR_FAULT):
            self.get_logger().error(text)
        elif decision in (Decision.SLOW_DOWN, Decision.RECOVERY_REQUIRED):
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)


def main(args=None):
    """Run the read-only prototype until shutdown."""
    rclpy.init(args=args)
    node = None
    try:
        node = SlipResponseNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
