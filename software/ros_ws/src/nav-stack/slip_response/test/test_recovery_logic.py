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


"""Unit tests for the ROS-independent dry-run decision logic."""

import unittest

from slip_response.recovery_logic import Decision, ResponseConfig, SlipResponseLogic


class TestSlipResponseLogic(unittest.TestCase):
    """Exercise freshness, hysteresis, escalation, and recovery latching."""

    def setUp(self):
        """Create a fresh logic instance for each test."""
        self.logic = SlipResponseLogic(ResponseConfig(), started_at_s=0.0)

    def sample(self, now_s, evaluating=True, slipping=False):
        """Deliver a fresh assessment and return the resulting decision."""
        self.logic.receive(evaluating, slipping, now_s)
        return self.logic.decide(now_s)

    def stop_episode(self):
        """Drive a continuous slip episode up to the stop decision."""
        self.assertEqual(self.sample(0.0, slipping=True), Decision.SLOW_DOWN)
        self.assertEqual(self.sample(1.0, slipping=True), Decision.SLOW_DOWN)
        self.assertEqual(self.sample(2.0, slipping=True), Decision.STOP)

    def test_waiting_then_fault_when_no_messages_arrive(self):
        """Detect input silence even without callbacks."""
        self.assertEqual(self.logic.decide(0.0), Decision.WAITING_FOR_DATA)
        self.assertEqual(self.logic.decide(2.0), Decision.SENSOR_FAULT)

    def test_normal_tracking(self):
        """Fresh evaluated healthy status produces normal operation."""
        self.assertEqual(self.sample(0.0), Decision.NORMAL)

    def test_initial_idle_is_not_normal_tracking(self):
        """An unevaluated assessment produces idle rather than tracking proof."""
        self.assertEqual(self.sample(0.0, evaluating=False), Decision.IDLE)

    def test_slip_escalates_to_stop_then_recovery_required(self):
        """Sustained slip produces decision escalation without executing actions."""
        self.stop_episode()
        self.assertEqual(self.sample(2.5, slipping=True), Decision.STOP)
        self.assertEqual(self.sample(3.0, slipping=True), Decision.RECOVERY_REQUIRED)

    def test_idle_does_not_clear_recovery_latch(self):
        """Stopping naturally makes input idle but is not successful recovery."""
        self.stop_episode()
        self.assertEqual(self.sample(2.5, evaluating=False), Decision.STOP)
        self.assertEqual(self.sample(3.0, evaluating=False), Decision.RECOVERY_REQUIRED)

    def test_healthy_input_does_not_clear_recovery_latch(self):
        """Recovery completion needs more than a non-slipping message."""
        self.stop_episode()
        self.assertEqual(self.sample(3.0), Decision.RECOVERY_REQUIRED)
        self.assertEqual(self.sample(4.5), Decision.RECOVERY_REQUIRED)

    def test_brief_slip_requires_healthy_hysteresis_before_normal(self):
        """Prevent speed request oscillations around the slip threshold."""
        self.sample(0.0, slipping=True)
        self.assertEqual(self.sample(0.2), Decision.SLOW_DOWN)
        self.assertEqual(self.sample(0.8), Decision.SLOW_DOWN)
        self.assertEqual(self.sample(1.3), Decision.NORMAL)

    def test_idle_preserves_warning_but_restarts_continuous_slip_timer(self):
        """Idle time cannot masquerade as sustained slip or successful recovery."""
        self.sample(0.0, slipping=True)
        self.assertEqual(self.sample(1.0, evaluating=False), Decision.SLOW_DOWN)
        self.assertEqual(self.sample(5.0, slipping=True), Decision.SLOW_DOWN)
        self.assertEqual(self.sample(7.0, slipping=True), Decision.STOP)

    def test_good_sample_between_ticks_breaks_continuous_slip(self):
        """Process intervening messages even when the timer has not ticked."""
        self.sample(0.0, slipping=True)
        self.logic.receive(True, False, 1.9)
        self.logic.receive(True, True, 1.95)
        self.assertEqual(self.logic.decide(2.0), Decision.SLOW_DOWN)

    def test_stale_input_takes_priority(self):
        """Old healthy status cannot keep the decision normal indefinitely."""
        self.sample(0.0)
        self.assertEqual(self.logic.decide(2.01), Decision.SENSOR_FAULT)

    def test_fault_does_not_discard_recovery_latch(self):
        """Restoring input after a fault does not silently resume navigation."""
        self.stop_episode()
        self.assertEqual(self.logic.decide(4.01), Decision.SENSOR_FAULT)
        self.assertEqual(self.sample(4.1), Decision.RECOVERY_REQUIRED)

    def test_fault_breaks_continuous_slip_history(self):
        """Do not count an input outage toward sustained slip."""
        self.sample(0.0, slipping=True)
        self.assertEqual(self.logic.decide(2.01), Decision.SENSOR_FAULT)
        self.assertEqual(self.sample(2.1, slipping=True), Decision.SLOW_DOWN)

    def test_receipt_gap_breaks_slip_history_even_without_timer_tick(self):
        """Do not count a stale gap if the timer was temporarily delayed."""
        self.sample(0.0, slipping=True)
        self.sample(1.0, slipping=True)
        self.assertEqual(self.sample(4.0, slipping=True), Decision.SLOW_DOWN)

    def test_receipt_gap_breaks_healthy_hysteresis(self):
        """A missing interval is not evidence of sustained healthy tracking."""
        self.sample(0.0, slipping=True)
        self.sample(0.2)
        self.assertEqual(self.sample(3.0), Decision.SLOW_DOWN)

    def test_confirmation_rejects_idle_slipping_and_stale_input(self):
        """Reject recovery completion without fresh evaluated healthy status."""
        self.stop_episode()
        with self.assertRaises(ValueError):
            self.logic.confirm_recovery_complete(2.0)
        self.sample(2.5, evaluating=False)
        with self.assertRaises(ValueError):
            self.logic.confirm_recovery_complete(2.5)
        self.sample(3.0)
        with self.assertRaises(ValueError):
            self.logic.confirm_recovery_complete(5.01)

    def test_explicit_confirmation_clears_recovery_latch(self):
        """A future executor can acknowledge a validated successful recovery."""
        self.stop_episode()
        self.sample(3.0)
        self.logic.confirm_recovery_complete(3.0)
        self.assertEqual(self.logic.decide(3.0), Decision.NORMAL)

    def test_invalid_config_is_rejected(self):
        """Reject unsafe or nonsensical timing parameters."""
        for value in (0.0, -1.0, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                ResponseConfig(status_timeout_s=value)

    def test_backward_time_is_rejected(self):
        """Require the monotonic clock used for freshness checks."""
        self.sample(1.0)
        with self.assertRaises(ValueError):
            self.logic.decide(0.9)


if __name__ == '__main__':
    unittest.main()
