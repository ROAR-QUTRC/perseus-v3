# Bucket ros2_control bench test

This is the procedure for bringing up the bucket's ros2_control stack
(`bucket.launch.py`, `payloads/BucketHardware`) on the real bucket for the first
time. Work through the phases in order. Each one assumes the previous one passed.

On the rover the bucket is a second hardware component under the drive's one
`controller_manager` (`perseus.launch.py payload:=bucket`). `bucket.launch.py` is
the bench version of the same thing without the drive: the same controllers, topic
and action names, so everything here carries straight over. Never run the two
together.

The stack sends the bucket position setpoints (`SET_POSITION`), and the firmware
drives flat out to each one and stops within 2 deg. **The firmware does not
watchdog position commands.** If the CAN link drops mid-move, the bucket keeps
driving towards the last setpoint. Keep a hand on the power cut for every phase
that moves the bucket.

The gamepad teleop (`bucket_driver`) always runs alongside as an operator override.
Moving a stick past its deadband deactivates the trajectory controller and hands the
bucket to the gamepad (`SET_SPEED`); `ros2 run payloads bucket_supervisor.py --rearm`
hands it back, the same command as after a fault.

## Joint conventions

These are the values ROS expects. `/joint_states_deg` shows them in degrees.

| Joint               | 0 deg                     | Positive            | URDF range               |
| ------------------- | ------------------------- | ------------------- | ------------------------ |
| `bucket_lift_joint` | arms at the highest point | down                | 0 to 71.4 deg            |
| `bucket_tilt_joint` | bucket level              | tipping down (dump) | -91.7 (curl) to 28.1 deg |
| `bucket_jaw_joint`  | jaws closed               | opening             | 0 to 48.0 deg            |

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
- A gamepad for `bucket_driver` teleop, used in phases 2, 4 and 8.
- A second person if possible: one on the power cut, one on the laptop.

Software, on the laptop:

```bash
devenv shell
cd software/ros_ws
colcon build --packages-up-to payloads description mission_bt_server
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
ros2 launch description view_perseus.launch.py live:=true   # second terminal
ros2 run mission_bt_server bucket_cli move --lift 45 --tilt -15 --jaw 30 --duration 3
```

Pass: `bucket_cli` prints `succeeded`, `/joint_states_deg` reads 45 / -15 / 30, and
RViz shows the bucket lowered, curled and open, with the rams attached. The rams
come from `bucket_linkage_broadcaster`. Always use `live:=true` against a running
stack: without it `view_perseus.launch.py` brings its own `robot_state_publisher` and
`joint_state_publisher`, which publish every joint at 0 over the real ones and make
the bucket flicker.

The same against the whole rover, mocked:
`ros2 launch perseus perseus.launch.py payload:=bucket use_mock_hardware:=true`.

To pose the model by hand in degrees instead, run
`ros2 launch description view_perseus.launch.py sliders:=true` and
`ros2 topic echo /joint_states_deg`. The sliders themselves are in radians.

## Phase 0b: the real hardware plugins on vcan (no bucket needed)

`rover_can_sim.py` stands in for the rover's CAN bus: the four drive VESCs and the
bucket board, with the bucket firmware's mode rules (the 1% `SET_SPEED` deadband,
flat-out position moves, no position watchdog). So this runs the real
`VescSystemHardware` and `BucketHardware`, end to end.

```bash
software/scripts/vcan-setup.sh vcan0                                   # once per boot
ros2 run payloads rover_can_sim.py --iface vcan0 --mode sweep --axis lift
ros2 launch perseus perseus.launch.py payload:=bucket can_bus:=vcan0 bucket_controller:=none
ros2 launch description view_perseus.launch.py live:=true
```

Pass: the arms sweep up and down with their rams, driven only by the `GET_ANGLE`
stream, and `/joint_states_deg` follows.

Then restart the simulator in follow mode (`rover_can_sim.py --iface vcan0`) and the
launch with the default controller, and run the mission behaviours:

```bash
ros2 run mission_bt_server bucket_cli move --lift 30
ros2 run mission_bt_server bucket_cli prepare
ros2 run mission_bt_server bucket_cli dump
ros2 run mission_bt_server bucket_cli dig --drive stub
```

Pass: each prints `succeeded` and the simulator's angles agree. Then the failures,
typed into the simulator:

- `drop lift_left` mid-move: the bucket stops (the simulator's banks go to
  `vel +0.0%`, and `bucket_trajectory_controller` goes inactive), while
  `diff_drive_base_controller` stays active with the wheels still on `/joint_states`.
  `bucket_supervisor` brings `BucketSystem` and the broadcasters back on its own.
  `restore`, then `ros2 run payloads bucket_supervisor.py --rearm` re-arms the
  trajectory controller.
- Ctrl-C the launch mid-move: the simulator shows a 2% then 0 `SET_SPEED` and the
  bank leaves position mode.
- Operator override: run `bucket_cli dump`, and part-way through publish a stick
  input in another terminal:
  `ros2 topic pub -r 20 /bucket_actuators actuator_msgs/msg/Actuators "{velocity: [0.05, 0.0, 0.0]}"`.
  `bucket_cli` fails with "operator override", the trajectory controller goes
  inactive, and the lift moves under `SET_SPEED`. Stop the publisher, then
  `ros2 run payloads bucket_supervisor.py --rearm`: the controller comes back
  holding where the bucket is.

## Phase 1: CAN link (bucket powered, no ROS)

With the bucket controller powered, watch `candump -L can0`. You should see:

| ID                                 | What                                  | Rate         |
| ---------------------------------- | ------------------------------------- | ------------ |
| `02000300`, `02000400`             | lift encoder L / R, `GET_ANGLE`       | every 50 ms  |
| `02000500`, `02000600`             | tilt encoder L / R                    | every 50 ms  |
| `02000700`, `02000800`             | jaw encoder L / R                     | every 50 ms  |
| `02000003`, `02000103`, `02000203` | lift / tilt / jaw bank `GET_POSITION` | every 50 ms  |
| `02000001`, `02000101`, `02000201` | bank `GET_CURRENT`                    | every 500 ms |

`GET_FAULT` (`02000000`, `02000100`, `02000200`) is not transmitted by the current
firmware, so don't expect it.

Expect zero `SET_SPEED` frames too (`02000002#0000` and so on, every 50 ms) once the
ROS stack is up: that is the teleop's idle stream, which the firmware ignores in
position mode.

The data is a little-endian int16 in tenths of a degree. For example, `2C01` is
0x012C = 300 = 30.0 deg.

Pass: all six encoder IDs are present at 50 ms, and each one's value changes when
you move that axis by hand or with teleop.

Record the raw angle of every encoder at rest. Left and right on the same axis
should agree to within a degree or two. A big difference means a skewed axis or a
mis-zeroed encoder, so fix that before going further.

## Phase 2: zero and direction (read-only)

Run the stack in calibration mode, which sends no position commands. The gamepad
teleop (`bucket_driver`) runs alongside, so the sticks move the bucket.

```bash
ros2 launch payloads bucket.launch.py controller:=none
ros2 topic echo /joint_states_deg
```

If the log complains, read the line:

- `No current encoder data for joint ...`: no `GET_ANGLE` frames are arriving.
  Recheck phase 1.
- `Refusing to command ... reads X deg, outside its limits [...]` (when a controller
  is spawned, phase 5 on): the firmware zero or direction doesn't match the table.
  Note X, fix `offset_deg` or `direction` in the xacro, rebuild `description`, and
  relaunch.

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
| ---- | ----------------- | ------------------------- | ---------- | --------- |
| lift |                   |                           |            |           |
| tilt |                   |                           |            |           |
| jaw  |                   |                           |            |           |

## Phase 3: RViz matches the bucket

Keep phase 2 running, then:

```bash
ros2 launch description view_perseus.launch.py live:=true
```

Pass: as you drive each axis with teleop, the model moves the same way by the same
amount, the rams stay attached, and nothing jitters.

## Phase 4: travel against the URDF limits

Still in calibration mode, drive each axis slowly end to end with teleop. Record
the readings at the mechanical ends.

| Axis | Min reading | Max reading | URDF range    |
| ---- | ----------- | ----------- | ------------- |
| lift |             |             | 0 to 71.4     |
| tilt |             |             | -91.7 to 28.1 |
| jaw  |             |             | 0 to 48.0     |

Pass: within a few degrees of the URDF range. If the real travel is smaller, update
the limits in `description/urdf/bucket.urdf.xacro`. Nothing clamps commands to those
limits (the controller_manager's limit enforcement is off), so goals must stay
inside the real travel.

From here on, keep your hands off the gamepad unless you mean to take over: any
stick input past the deadband takes the bucket from the trajectory controller.

## Phase 5: first closed-loop move (lift only)

```bash
ros2 launch payloads bucket.launch.py controller:=bucket_lift_controller
ros2 topic echo /joint_states_deg      # note the lift reading, P deg
```

Send small moves relative to P. Start with P + 3 deg, then return to P:

```bash
ros2 run mission_bt_server bucket_cli move --lift <deg> --duration 3 \
  --server /bucket_lift_controller/follow_joint_trajectory
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
   Pass: the bucket stops, and `candump` shows `02000002#8F02` then `02000002#0000`,
   and the same for `02000102` and `02000202`. The 2% (`8F02`) is what takes a bank
   out of position mode: the firmware ignores a plain zero there.
2. **Encoder unplugged.** Unplug the lift-left encoder while the lift is holding.
   Known firmware gap: the board keeps broadcasting the last cached angle, so ROS
   will **not** notice. Record what the bucket does; this is the reason to fix
   `motor_parameter_group.cpp` in firmware.
3. **CAN unplugged.** Unplug the CAN cable while the lift is holding. Pass: the
   stack logs `Encoder for joint ... is stale - stopping` within about 100 ms. The
   stop command can't reach the board with the cable out, so the bucket holds its
   last setpoint until the power is cut. Plug it back in: `bucket_supervisor`
   restores the encoders on `/joint_states`, and
   `ros2 run payloads bucket_supervisor.py --rearm` re-arms.

## Phase 7: tilt and jaws

Phases 0 to 4 must have passed for tilt and jaws too: zero, direction and travel
all checked. Lift passed phases 5 and 6.

### 7a: firmware drive direction (per axis)

The firmware's position loop drives every bank with the sign in
`kDriveDirection` (`firmware/excavation-bucket/include/motor_bank.hpp`; one value
for all three banks on this branch - `feat/master-encoder-firmware` has a
`POSITION_DIRECTION` per bank in `excavation_config.hpp`). It must be `+1` if a
positive `SET_SPEED` increases that bank's encoder angle, and `-1` if it decreases
it. Lift is confirmed as `+1`. Tilt and jaws are not confirmed yet. If the sign is
wrong, the bank drives away from its target until it hits the end stop.

`direction` in the xacro does not fix this. That param only maps the firmware
angle onto the URDF joint. The firmware's sign is the motor polarity relative to
the encoder.

For tilt and then jaws:

1. Run calibration mode (`controller:=none`; the teleop runs alongside), and watch
   `candump -L can0`.
2. Move the axis slowly with teleop. In `candump`, compare the sign of the bank's
   `SET_SPEED` frame (`02000102` tilt, `02000202` jaw) with the change in the
   raw `GET_ANGLE` value for that axis.
3. If a positive `SET_SPEED` makes the angle go down, that axis needs a drive
   direction of `-1` in the firmware.
4. Rebuild and flash the firmware.

| Axis | Raw angle with + speed | Drive direction |
| ---- | ---------------------- | --------------- |
| lift | up                     | +1              |
| tilt |                        |                 |
| jaw  |                        |                 |

### 7b: single-axis moves

Repeat phases 5 and 6 for each axis with its own controller. Each one claims
only its own joint, so the other two banks get no `SET_POSITION` frames.

| Axis | Controller               | Joint               | `SET_POSITION` ID |
| ---- | ------------------------ | ------------------- | ----------------- |
| tilt | `bucket_tilt_controller` | `bucket_tilt_joint` | `02000104`        |
| jaw  | `bucket_jaw_controller`  | `bucket_jaw_joint`  | `02000204`        |

```bash
ros2 launch payloads bucket.launch.py controller:=bucket_tilt_controller
ros2 run mission_bt_server bucket_cli move --tilt <deg> --duration 3 \
  --server /bucket_tilt_controller/follow_joint_trajectory
```

For the first goal, send a move of 3 deg, and keep your hand on the power cut.
If the axis moves away from the target, cut the power. That means
`DRIVE_DIRECTION` is wrong (see 7a).

### 7c: full controller

When all three axes pass alone, run `controller:=bucket_trajectory_controller`.
This controller claims all three joints. Every bank then receives a
`SET_POSITION` frame each cycle. A joint that is not in the goal holds its
current position.

1. Send single-joint goals, one axis at a time. Check that the other two axes
   hold still.
2. Send a goal for all three joints, with small moves on each:

   ```bash
   ros2 run mission_bt_server bucket_cli move --lift <deg> --tilt <deg> --jaw <deg> --duration 5
   ```

3. Repeat phase 6 check 1 (Ctrl-C mid-move). All three banks must stop.

Pass: every goal returns `SUCCEEDED`, and RViz matches the bucket.

`perseus/launch/perseus.launch.py` already defaults `bucket_controller` to
`bucket_trajectory_controller`. Until 7c passes, pass a single-axis controller
or `bucket_controller:=none` explicitly.

## Phase 8: operator override

With `controller:=bucket_trajectory_controller`, start a slow move
(`bucket_cli move --lift <deg> --duration 15`) and nudge a stick part-way through.

Pass:

- `bucket_cli` fails with "operator override", and `ros2 control list_controllers`
  shows `bucket_trajectory_controller` inactive.
- The bucket follows the stick, and stops when you let go.
- `candump` shows no `SET_POSITION` (`02000004`) frames after the takeover.
- `bucket_supervisor.py --rearm` succeeds and the bucket stays where you left it,
  with no jump.
  A following `bucket_cli move` works.
- A gentle stick touch below the deadband does not take over.

Then the behaviours: `bucket_cli prepare`, `bucket_cli dump`, and on the rover with
nav2 up, `bucket_cli dig`.

## After the session

- Commit any `offset_deg`, `direction` and limit changes, with the recorded tables.
- File the firmware follow-ups: broadcast `GET_ANGLE` only while the reading is
  fresh, and add a `GET_FAULT` transmission.
