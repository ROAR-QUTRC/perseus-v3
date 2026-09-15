# Slip-response dry-run prototype

This is **not a traction controller or autonomous recovery executor**. It
subscribes to `interfaces/msg/MobilityStatus` and logs hypothetical decisions.
It has no command publishers, recovery services, action clients, Nav2 calls,
or motor commands. Standard ROS node logging and parameter interfaces remain.

## Files

- Copy `slip_response/recovery_logic.py` and `slip_response/response_node.py`
  into the generated ROS package's inner `slip_response` directory.
- Copy `test/test_recovery_logic.py` into the package's `test` directory.
- Add this console script in the generated `setup.py`:

```python
'response_node = slip_response.response_node:main',
```

Set the package description to "Read-only slip-response decision prototype for
the mobility watchdog" and complete the maintainer information. This prototype
uses MIT to match the existing watchdog package. Include `README.md` and
`LICENSE` in `setup.py`'s package-share data files. Generated ROS lint tests
retain their existing notices and are not replaced by these logic tests.

From the generated ROS package root, ROS's license tool can add missing MIT
headers to the new source/test files without altering existing notices:

```bash
ament_copyright --add-missing "Omar Kassab" mit slip_response test
```

Only run that command on these package-local directories, not the repository
root. Confirm all changes in `git diff` before committing.

## Decisions and timings

- `WAITING_FOR_DATA`: no input yet, before the freshness timeout.
- `IDLE`: the watchdog is not evaluating; idle is not tracking/recovery proof.
- `NORMAL`: a fresh evaluated assessment does not report slip.
- `SLOW_DOWN`: sustained slip reported by the watchdog; hypothetical slowdown.
- `STOP`: evaluated slip persists for another `stop_after_s` (default 2 s).
- `RECOVERY_REQUIRED`: after `recovery_delay_s` (default 1 s) in the stopped
  episode, an autonomous recovery executor would need to act. Nothing runs.
- `SENSOR_FAULT`: no input for `status_timeout_s` (default 2 s).

Defaults are demonstration parameters, **not rover tuning recommendations**.
They are additional to the watchdog's own detection window and grace period.
The freshness timeout must exceed the configured watchdog reporting interval.
Pre-stop clearing needs fresh evaluated healthy input for `clear_after_s`
(default 1 s). Idle pauses/reset continuous slip timing but preserves the
warning. A stopped episode stays latched even if later messages become idle
or apparently healthy. A future autonomous executor must explicitly confirm
successful recovery with `confirm_recovery_complete`; the ROS node deliberately
never calls it. Restarting this prototype resets its demonstration state; this
is not an operational human-help fallback or a production recovery mechanism.

Receipt freshness uses a monotonic clock and the timer can detect silence even
when callbacks stop. It does not independently validate sensor source stamps,
odometry covariance, frames, localization quality, or actual wheel velocity.
`is_slipping` indicates a command/motion mismatch, not confirmed wheel spin.

## Test in WSL

From the ROS package's root:

```bash
python3 -m unittest discover -s test -p 'test_recovery_logic.py' -v
```

Build interfaces and this package from the ROS workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to slip_response
source install/setup.bash
ros2 run slip_response response_node
```

In another sourced WSL terminal, with the real watchdog and motor/controller
nodes stopped, publish mock status messages. Use a private test topic if any
other ROS system could share the same network/domain.

```bash
ros2 run slip_response response_node --ros-args -p status_topic:=/slip_response/test_status
ros2 topic pub -r 10 /slip_response/test_status interfaces/msg/MobilityStatus "{is_evaluating: true, is_slipping: true}"
```

Expected log transitions: `SLOW_DOWN`, about 2 s later `STOP`, about another
1 s later `RECOVERY_REQUIRED`. These are logs, **not executed actions**. Stop
the publisher: after the freshness timeout, expect `SENSOR_FAULT`. Restart the
prototype for independent demonstrations of normal/idle assessments.

## Remaining work before controlling a rover

Review ROS/package compatibility and license/metadata; run generated lint
checks; validate thresholds and timestamp handling; define autonomous retry
limits, mission-failure behavior, and recovery-success criteria; integrate
through the existing navigation/velocity arbitration system with obstacle and
localization checks. Do not issue commands directly from the watchdog, bypass
safety ownership, blindly reverse/spin, or assume stopping freed the rover.
