# About this Package

This package is intended to contain payload-specific code for Perseus. `*_driver` packages are intended to provide an interface between real hardware and ROS topics/services/actions - however, since they should make use of the most applicable ROS message types for their specific hardware, they may not be immediately useful to an end user.

This is where `*_controller` nodes come in.
They are intended to provide an interface between your _control_ messages (eg, `TwistStamped`) and the _driver nodes_ (which might take something like an `Actuators` message).
This isolates responsibilities - the driver nodes can focus on providing a transparent interface to hardware, and the controller nodes can focus on taking useful _input_ methods and converting that to hardware control.

## Arm

```bash
ros2 launch payloads servo_sim.launch.py      # mock hardware + RViz; flip the constants at the top for real servos
ros2 launch payloads servo_sim.launch.py use_mock_hardware:=false mock_servo_ids:=4,5,6 # Real hardware


ros2 launch payloads arm_teleop_sim.launch.py # same stack plus DualSense teleop
ros2 run payloads keyboard_control            # keyboard teleop, second shell
```

`scripts/generate_ikfast_plugin.sh` regenerates `arm_ikfast_plugin`; run it only when the geometry in `config/arm_model.urdf` changes.

### Real servos

`use_mock_hardware:=false` loads `payloads/DynamixelServos`, a ros2_control
plugin driving the arm over Dynamixel Protocol 2.0. Port, baud rate and the
mocked-servo list are `<param>`s in `config/arm.ros2_control.xacro`.

```bash
ros2 launch payloads servo_sim.launch.py use_mock_hardware:=false
ros2 launch payloads servo_sim.launch.py use_mock_hardware:=false mock_servo_ids:=4,5,6
```

`mock_servo_ids` (comma-separated, no spaces) simulates those IDs in software so
the arm runs with servos not yet wired. It defaults to `6`: the gripper mechanism
is unbuilt, so its `prismatic_scale` and closed-position `offset` in the xacro are
placeholders.

The plugin sets the FTDI adapter's latency timer to 1 ms itself on every open
(the 16 ms Linux default would cap the 100 Hz loop near 30 Hz), so no host setup
is needed. A udev rule doing the same system-wide is optional and also helps
`screen`/`minicom`:

```
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```

ls -l /dev/ttyUSB* /dev/ttyACM*
ls -l /dev/serial/by-id/

lsusb -t

udevadm info -a -n /dev/ttyUSB0 | head -40 # vendor/product ids for udev rules
stty -F /dev/ttyUSB0 -a # current baud/line settings
sudo fuser -v /dev/ttyUSB0 # what process is holding it
