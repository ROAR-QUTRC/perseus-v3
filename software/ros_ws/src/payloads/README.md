# payloads

Payload-specific nodes for Perseus: `*_driver` nodes talk to hardware, `*_controller` nodes turn control messages into driver commands.

## Arm

```bash
ros2 launch payloads servo_sim.launch.py        # mock hardware + RViz
ros2 launch payloads arm_hardware.launch.py     # real servos 1-3, wrist and gripper mocked, DualSense teleop
ros2 launch payloads dynamixel_test.launch.py   # bus + ros2_control only, no MoveIt
ros2 run payloads keyboard_control              # keyboard teleop, second shell
```

- `use_mock_hardware:=false` loads `payloads/DynamixelServos` (Protocol 2.0, 1 Mbaud, extended position mode). Port, baud and `mock_servo_ids` (IDs simulated in software; comma-separated, no spaces) live in `config/arm.ros2_control.xacro`.
- The gripper is a 3-pin PWM servo, not a Dynamixel: keep ID 6 in `mock_servo_ids` until it has its own driver.
- `scripts/generate_ikfast_plugin.sh` regenerates `arm_ikfast_plugin`; needed only when `config/arm_model.urdf` geometry changes.

### Servo setup and calibration

- Bare servo: the horn at its alignment mark (count 2048) reads as joint 0. Put the gearbox ratio in the joint's `<mechanical_reduction>` (negative flips direction); the joint `<offset>` only seeds the first run.
- Assembled joint: move it to a known angle, stop the controller, tell the plugin the true angle, restart the controller. The offset persists in `~/.ros/dynamic_offset_transmissions/<joint>.txt`; delete the file to start from 0 again.

```bash
ros2 control set_controller_state servo_controller inactive
ros2 service call /dynamixel_servos/adjust_transmission_offsets hector_transmission_interface_msgs/srv/AdjustTransmissionOffsets \
  "{external_joint_measurements: {name: [elbow], position: [1.571]}}"
ros2 control set_controller_state servo_controller active
```

- Power loss resets a servo's turn count. Park geared joints within half a servo turn (pi / reduction) of `initial_positions.yaml` before power-up; restarts without power loss recover the exact frame, and a servo that reboots mid-run is re-synced and re-torqued automatically.
