import pytest

from teleop.speed_control import SpeedControl


def make_control():
    return SpeedControl(0.25, 0.50, 0.25, 1.00)


def test_starts_at_default_and_persists_at_neutral():
    control = make_control()
    assert control.scale == 0.50
    assert not control.update(0.0)
    assert control.scale == 0.50


def test_up_and_down_step_once_per_press():
    control = make_control()
    assert control.update(1.0)
    assert control.scale == 0.75
    assert not control.update(1.0)
    assert control.scale == 0.75
    assert not control.update(0.0)
    assert control.update(-1.0)
    assert control.scale == 0.50


def test_scale_is_clamped_to_configured_bounds():
    control = make_control()
    for _ in range(5):
        control.update(1.0)
        control.update(0.0)
    assert control.scale == 1.00

    for _ in range(8):
        control.update(-1.0)
        control.update(0.0)
    assert control.scale == 0.25


@pytest.mark.parametrize(
    "values",
    [
        (0.0, 0.5, 0.25, 1.0),
        (0.25, 0.2, 0.25, 1.0),
        (0.25, 0.5, 0.0, 1.0),
        (0.25, 0.5, 0.25, 1.1),
    ],
)
def test_rejects_unsafe_configuration(values):
    with pytest.raises(ValueError):
        SpeedControl(*values)
