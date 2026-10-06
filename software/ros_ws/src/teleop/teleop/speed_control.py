"""Persistent, bounded operator speed selection."""


class SpeedControl:
    """Convert D-pad direction changes into a bounded persistent scale."""

    def __init__(self, minimum, default, increment, maximum):
        if minimum <= 0.0:
            raise ValueError("minimum speed scale must be greater than zero")
        if maximum > 1.0:
            raise ValueError("maximum speed scale must not exceed 1.0")
        if minimum > default or default > maximum:
            raise ValueError("default speed scale must be within min/max bounds")
        if increment <= 0.0:
            raise ValueError("speed scale increment must be greater than zero")

        self.minimum = float(minimum)
        self.maximum = float(maximum)
        self.increment = float(increment)
        self.scale = float(default)
        self._previous_direction = 0

    def update(self, axis_value):
        """Apply one step on each neutral-to-up/down transition."""
        direction = 1 if axis_value > 0.5 else -1 if axis_value < -0.5 else 0
        changed = False

        if direction != 0 and self._previous_direction == 0:
            requested = self.scale + direction * self.increment
            selected = min(self.maximum, max(self.minimum, requested))
            if selected != self.scale:
                self.scale = selected
                changed = True

        self._previous_direction = direction
        return changed
