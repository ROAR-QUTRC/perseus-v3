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


"""
ROS-independent, dry-run slip-response state machine.

This module produces decisions only. It cannot drive, cancel navigation,
perform a recovery, or confirm that a physical recovery occurred.
"""

from dataclasses import dataclass
from enum import Enum
from math import isfinite
from typing import Optional


class Decision(str, Enum):
    """Possible supervisory decisions; none represents an executed action."""

    WAITING_FOR_DATA = 'WAITING_FOR_DATA'
    IDLE = 'IDLE'
    NORMAL = 'NORMAL'
    SLOW_DOWN = 'SLOW_DOWN'
    STOP = 'STOP'
    RECOVERY_REQUIRED = 'RECOVERY_REQUIRED'
    SENSOR_FAULT = 'SENSOR_FAULT'


@dataclass(frozen=True)
class ResponseConfig:
    """Prototype timings in seconds, not validated rover tuning values."""

    status_timeout_s: float = 2.0
    stop_after_s: float = 2.0
    recovery_delay_s: float = 1.0
    clear_after_s: float = 1.0

    def __post_init__(self):
        """Reject invalid timings rather than silently accepting them."""
        for name, value in vars(self).items():
            if not isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and greater than zero')


class SlipResponseLogic:
    """
    Interpret fresh watchdog assessments without controlling anything.

    A stopped episode stays latched until a future autonomous recovery
    component explicitly confirms completion. Idle or healthy messages alone
    do not imply that a recovery has happened.
    """

    def __init__(self, config: ResponseConfig, started_at_s: float):
        """Create the state machine using a monotonic clock supplied by callers."""
        if not isfinite(started_at_s):
            raise ValueError('started_at_s must be finite')
        self.config = config
        self._started_at_s = started_at_s
        self._clock_s = started_at_s
        self._last_received_s: Optional[float] = None
        self._is_evaluating = False
        self._is_slipping = False
        self._episode_active = False
        self._slip_since_s: Optional[float] = None
        self._clear_since_s: Optional[float] = None
        self._stopped_at_s: Optional[float] = None

    def _check_time(self, now_s: float):
        if not isfinite(now_s) or now_s < self._clock_s:
            raise ValueError('time must be finite and monotonic')
        self._clock_s = now_s

    def receive(self, is_evaluating: bool, is_slipping: bool, now_s: float):
        """Cache a newly received watchdog assessment."""
        self._check_time(now_s)
        if (
            self._last_received_s is not None
            and now_s - self._last_received_s > self.config.status_timeout_s
        ):
            self._slip_since_s = None
            self._clear_since_s = None
        self._last_received_s = now_s
        self._is_evaluating = is_evaluating
        self._is_slipping = is_slipping

        # Break continuous histories at receipt, even if a brief idle/good
        # message arrives between two timer ticks.
        if not is_evaluating:
            self._slip_since_s = None
            self._clear_since_s = None
        elif is_slipping:
            self._clear_since_s = None
            self._episode_active = True
            if self._slip_since_s is None:
                self._slip_since_s = now_s
        else:
            self._slip_since_s = None
            if self._episode_active and self._clear_since_s is None:
                self._clear_since_s = now_s

    def _has_fresh_status(self, now_s: float) -> bool:
        return (
            self._last_received_s is not None
            and now_s - self._last_received_s <= self.config.status_timeout_s
        )

    def decide(self, now_s: float) -> Decision:
        """Return a decision; a timer must call this even when input stops."""
        self._check_time(now_s)
        if self._last_received_s is None:
            if now_s - self._started_at_s < self.config.status_timeout_s:
                return Decision.WAITING_FOR_DATA
            return Decision.SENSOR_FAULT

        if not self._has_fresh_status(now_s):
            self._slip_since_s = None
            self._clear_since_s = None
            return Decision.SENSOR_FAULT

        # Preserve the recovery latch across idle, apparently healthy, and
        # temporarily stale assessments. A fault takes priority above.
        if self._stopped_at_s is not None:
            if now_s - self._stopped_at_s >= self.config.recovery_delay_s:
                return Decision.RECOVERY_REQUIRED
            return Decision.STOP

        if not self._is_evaluating:
            return Decision.SLOW_DOWN if self._episode_active else Decision.IDLE

        if self._is_slipping:
            if self._slip_since_s is None:
                self._slip_since_s = now_s
            if now_s - self._slip_since_s >= self.config.stop_after_s:
                self._stopped_at_s = now_s
                return Decision.STOP
            return Decision.SLOW_DOWN

        if self._episode_active:
            if self._clear_since_s is None:
                self._clear_since_s = now_s
            if now_s - self._clear_since_s < self.config.clear_after_s:
                return Decision.SLOW_DOWN
            self._episode_active = False
            self._clear_since_s = None
        return Decision.NORMAL

    def confirm_recovery_complete(self, now_s: float):
        """
        Accept a future recovery executor's confirmation, not just idle data.

        The caller must establish actual recovery success. Fresh, evaluated,
        non-slipping input is an additional necessary condition, not proof
        by itself. The read-only ROS node never calls this method.
        """
        self._check_time(now_s)
        if (
            not self._has_fresh_status(now_s)
            or not self._is_evaluating
            or self._is_slipping
        ):
            raise ValueError('recovery confirmation needs a fresh healthy assessment')
        self._stopped_at_s = None
        self._episode_active = False
        self._slip_since_s = None
        self._clear_since_s = None
