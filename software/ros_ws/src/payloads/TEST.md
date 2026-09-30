# Bucket ros2_control bench test

This is the procedure for bringing up the bucket's ros2_control stack
(`bucket.launch.py`, `payloads/BucketHardware`) on the real bucket for the first
time. Work through the phases in order. Each one assumes the previous one passed.

The stack sends the bucket position setpoints (`SET_POSITION`), and the firmware's
own PID drives to them. **The firmware does not watchdog position commands.** If
the CAN link drops mid-move, the bucket keeps driving towards the last setpoint.
Keep a hand on the power cut for every phase that moves the bucket.

## Joint conventions

These are the values ROS expects. `/payloads/joint_states_deg` shows them in
degrees.

| Joint | 0 deg | Positive | URDF range |
|---|---|---|---|
| `bucket_lift_joint` | arms at the highest point | down | 0 to 71.4 deg |
| `bucket_tilt_joint` | bucket level | tipping down (dump) | -91.7 (curl) to 28.1 deg |
| `bucket_jaw_joint` | jaws closed | opening | 0 to 48.0 deg |

The firmware reports each encoder in its own frame (its calibrated zero). If that
frame doesn't match the table, set `offset_deg` and `direction` for the joint in
`description/ros2_control/bucket.ros2_control.xacro`:

```
joint_deg = direction * (firmware_deg - offset_deg)
```

## What to have ready

Hardware:

- Bucket mounted on a stand or the rover, clear of people and objects over its
  full travel. Lift sweeps about 71 deg and tilt about 120 deg.
- Bench supply with a current limit, or the rover battery with its e-stop working.
  A power cut within arm's reach.
- USB-CAN adapter wired to the bucket controller board, with the bus terminated.
- Bucket controller flashed with the current `firmware/excavation-bucket`, and all
  six encoder boards connected. Magnets should be seated. The encoder status LEDs
  show whether each magnet is detected.
- A way to measure angle: a phone inclinometer app or a protractor. A marker to
  mark the zero poses.
- A gamepad for `bucket_driver` teleop, used in phases 2 and 4.
- A second person if possible: one on the power cut, one on the laptop.

Software, on the laptop:

```bash
devenv shell
cd software/ros_ws
colcon build --packages-up-to payloads description
source install/setup.bash
ros2 pkg prefix joint_trajectory_controller   # must print a path
```

Bring up `can0` the way you normally do for the rover, then check it:

```bash
ip -br link show can0          # must be UP
```

Keep a spare terminal running `candump -L can0` for the whole session. It is the
ground truth for what is going on the bus.

## Phase 0: software dry run (no bucket needed)

This has already passed on the dev machine. Repeat it after any code change.

```bash
ros2 launch payloads bucket.launch.py hardware_plugin:=mock_components/GenericSystem
ros2 launch description view_perseus.launch.py            # second terminal
ros2 action send_goal /payloads/bucket_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [bucket_lift_joint, bucket_tilt_joint, bucket_jaw_joint],
    points: [{positions: [0.7854, -0.2618, 0.5236], time_from_start: {sec: 3}}]}}"
```

Pass: the goal returns `SUCCEEDED`, `/joint_states_deg` reads 45 / -15 / 30, and
RViz shows the bucket lowered, curled and open, with the rams attached.

To pose the model by hand in degrees instead, run
`ros2 launch description view_perseus.launch.py sliders:=true` and
`ros2 topic echo /joint_states_deg`. The sliders themselves are in radians.

## Phase 1: CAN link (bucket powered, no ROS)

With the bucket controller powered, watch `candump -L can0`. You should see:

| ID | What | Rate |
|---|---|---|
| `02000300`, `02000400` | lift encoder L / R, `GET_ANGLE` | every 50 ms |
| `02000500`, `02000600` | tilt encoder L / R | every 50 ms |
| `02000700`, `02000800` | jaw encoder L / R | every 50 ms |
| `02000003`, `02000103`, `02000203` | lift / tilt / jaw bank `GET_POSITION` | every 50 ms |
| `02000001`, `02000101`, `02000201` | bank `GET_CURRENT` | every 500 ms |

`GET_FAULT` (`02000000`, `02000100`, `02000200`) is not transmitted by the current
firmware, so don't expect it.

The data is a little-endian int16 in tenths of a degree. For example, `2C01` is
0x012C = 300 = 30.0 deg.

Pass: all six encoder IDs are present at 50 ms, and each one's value changes when
you move that axis by hand or with teleop.

Record the raw angle of every encoder at rest. Left and right on the same axis
should agree to within a degree or two. A big difference means a skewed axis or a
mis-zeroed encoder, so fix that before going further.

## Phase 2: zero and direction (read-only)

Run the stack in calibration mode, which sends no commands. Start teleop alongside
it, with the gamepad.

```bash
ros2 launch payloads bucket.launch.py controller:=none
ros2 topic echo /payloads/joint_states_deg
```

If activation fails, read the log line:

- `No encoder data for joint ...`: no `GET_ANGLE` frames arrived. Recheck phase 1.
- `reads X deg, outside its limits [...]`: the firmware zero or direction doesn't
  match the table. Note X, fix `offset_deg` or `direction` in the xacro, rebuild
  `description`, and relaunch.

For each axis:

1. Drive it with teleop to its 0 deg pose: lift at the top, tilt level, jaw closed.
   It should read 0 (within about 1 deg).
2. Drive it in its positive direction: lift down, tilt dumping, jaw opening. The
   reading should increase.
3. If it decreases, set `direction: -1`. If 0 is off, set `offset_deg` to the
   firmware angle at the zero pose.

`bucket_positions.yaml` has lift "down" at 340 deg firmware, which is -20 deg. That
suggests lift may currently read negative going down, so check that one first.

| Axis | Reading at 0 pose | Increases in + direction? | offset_deg | direction |
|---|---|---|---|---|
| lift | | | | |
| tilt | | | | |
| jaw | | | | |

## Phase 3: RViz matches the bucket

Keep phase 2 running, then:

```bash
ros2 launch description view_perseus.launch.py     # sliders off, the default
```

Pass: as you drive each axis with teleop, the model moves the same way by the same
amount, the rams stay attached, and nothing jitters.

## Phase 4: travel against the URDF limits

Still in calibration mode, drive each axis slowly end to end with teleop. Record
the readings at the mechanical ends.

| Axis | Min reading | Max reading | URDF range |
|---|---|---|---|
| lift | | | 0 to 71.4 |
| tilt | | | -91.7 to 28.1 |
| jaw | | | 0 to 48.0 |

Pass: within a few degrees of the URDF range. If the real travel is smaller, update
the limits in `description/urdf/bucket.urdf.xacro`. Commands are clamped to those
limits, so they must never exceed the real travel.

Stop teleop (`bucket_driver`) now. It must not run from here on.

## Phase 5: first closed-loop move (lift only)

```bash
ros2 launch payloads bucket.launch.py controller:=bucket_lift_controller
ros2 topic echo /payloads/joint_states_deg      # note the lift reading, P deg
```

Send small moves relative to P. Convert degrees to radians (deg x 0.017453). Start
with P + 3 deg, then return to P:

```bash
ros2 action send_goal /payloads/bucket_lift_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [bucket_lift_joint],
    points: [{positions: [<rad>], time_from_start: {sec: 3}}]}}"
```

Check:

- `candump` shows `02000004#....` with the target in tenths of a degree. For
  example, 35.0 deg is `5E01`. There are no `02000104` / `02000204` frames: tilt
  and jaw are not commanded.
- The lift moves the right way and stops within 2 deg of the target.
- The goal returns `SUCCEEDED`. `ABORTED ... goal_time_tolerance` means it didn't
  reach the target within 3 s of the planned end. That is either a slow or stuck
  axis, or a wrong direction or offset.

Then try larger moves: 10 deg, then most of the range, at a slow
`time_from_start` (for example 10 s for 30 deg).

## Phase 6: stop and failure behaviour (lift only)

Run each check with the power cut in hand.

1. **Ctrl-C mid-move.** Send a long, slow goal, then Ctrl-C the launch halfway.
   Pass: the bucket stops, and `candump` shows `02000002#0000`, `02000102#0000`
   and `02000202#0000` (zero `SET_SPEED`).
2. **Encoder unplugged.** Unplug the lift-left encoder while the lift is holding.
   Known firmware gap: the board keeps broadcasting the last cached angle, so ROS
   will **not** notice. Record what the bucket does; this is the reason to fix
   `motor_parameter_group.cpp` in firmware.
3. **CAN unplugged.** Unplug the CAN cable while the lift is holding. Pass: the
   stack logs `Encoder for joint ... is stale - stopping` within about 100 ms. The
   stop command can't reach the board with the cable out, so the bucket holds its
   last setpoint until the power is cut.

## Phase 7: tilt and jaws

Do this only once firmware position mode is confirmed for the tilt and jaw banks
(`bucket_positions.yaml` has only lift enabled today). Repeat phases 5 and 6 using
`controller:=bucket_trajectory_controller` and single-joint goals, one axis at a
time.

## After the session

- Commit any `offset_deg`, `direction` and limit changes, with the recorded tables.
- File the firmware follow-ups: broadcast `GET_ANGLE` only while the reading is
  fresh, and add a `GET_FAULT` transmission.
