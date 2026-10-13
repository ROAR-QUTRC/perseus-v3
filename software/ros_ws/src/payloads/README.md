# About this Package

This package is intended to contain payload-specific code for Perseus. `*_driver` packages are intended to provide an interface between real hardware and ROS topics/services/actions - however, since they should make use of the most applicable ROS message types for their specific hardware, they may not be immediately useful to an end user.

This is where `*_controller` nodes come in.
They are intended to provide an interface between your _control_ messages (eg, `TwistStamped`) and the _driver nodes_ (which might take something like an `Actuators` message).
This isolates responsibilities - the driver nodes can focus on providing a transparent interface to hardware, and the controller nodes can focus on taking useful _input_ methods and converting that to hardware control.

## The excavation bucket

On the rover the bucket is a second ros2_control hardware component
(`payloads/BucketHardware`) under the drive's one `controller_manager`, added by
`perseus.launch.py payload:=bucket`. Its controllers live in
`config/bucket_controller.yaml`:

- `bucket_joint_state_broadcaster` and `bucket_linkage_broadcaster` are read-only.
  They put lift, tilt and jaw from the encoder stream, plus the ten ram joints
  solved from them, on `/joint_states`.
- `bucket_trajectory_controller` (or a single-axis one) takes the mission's
  `MoveBucket` goals and sends them as `SET_POSITION` frames.

A bucket fault deactivates only the bucket's side; the drive keeps running.
`bucket_supervisor.py` brings the read-only side back by itself. Re-arming the
trajectory controller is always a person's call.

`bucket_driver` (the gamepad teleop) runs alongside as the operator's override. A
stick input takes the bucket from the trajectory controller.

`ros2 run payloads bucket_supervisor.py --rearm` is the one way back to autonomy,
after a fault or a takeover alike: it recovers the hardware and hands the bucket to
the trajectory controller (through `/bucket/rearm` when the operator has it).

`launch/bucket.launch.py` is the same stack without the drive, for the bench.
`scripts/rover_can_sim.py` simulates the rover's CAN bus on vcan, and
`mission_bt_server`'s `bucket_cli` runs the mission's bucket behaviours. See
`TEST.md` for the bring-up procedure.
