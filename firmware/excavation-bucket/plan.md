# The plan for ENC

## Comms between Excavator driver (ExDriver) and ROS2
- Velocity <-
- Position <-
- Speed <-
- Current position ->
- Board status ->
    - Encoder pos 
    - Encoder status
    - ExDriver status (corrupt frames, etc)
- Actuator current ->
- TODO: Need to figure out how homing will be done

ExDriver will receive position and velocity commands, velocity overrrides position and cancels that operation. Basically when running autonomy ROS2 will send target positions for the bucket to assume, feedback control is handled by the bucket driver.
Velocity overrides position as this will handle teleop, when Mozz is driving manually he will use macros that will assume set positions, but that motion can be cancelled and non feedback control will take over until the macro is toggled. So one will take over the other, when velocity is being sent a position will cancel velocity, when position is sent a velocity command other than zero or over a threshold, will take precendence.

The speed of the actuators will be sent via ROS2 as well, with some physical limits set. Autonomy will set this value, and will be overriden when in teleop mode by the pot on the taranis transmitter.

The ExDriver will send its status to ROS2 to show smoothed position values, status of both driver and encoder, and any link degradation on both systems.

ExDriver will then send current of actuators, note these are banked so its a ballpark figure on stalling, especially when compared to encoder velocities locally on the ExDriver.

## Comms between encoder and ExDriver
- Angle -> 
- Status ->
    - CRC/dropped frame error, DIR.
- Discovery <-
- Zero <-
- HeartBeat <-

ExDriver will maintain a 20-50ms heartbeat to the encoders on register 0 (broadcast) so every encoder knows things are good, the encoders dont respond to the heartheart beat, but will set their status to ERR, and visually flash red. The ExDriver will also cease the heartbeat if there is any other error to visually indicate so.
The life of an encoder is gathered through consecutive readings of the encoders, an unstable link will cause the encoder to report a degradated link that ExDriver can choose to latch or roll off after 10 successful callbacks. 

Angle is received raw in degrees upon request from ExDriver as fast as it dictates, so far 50Hz is the sample rate Mozz designed it to but a lower one can be chosen, these won't be directly sent to ROS2 as to not clog the CAN bus with verbosity. 

Status is requested and details encoder's errors, and other info we can decide.

Discovery is to physically illuminate a specific encoder, it will block other LED statuses, its so a problematic or partiular encoder can be discovered. Its a bright white light that will shine through a clear portion of the housing.

Zero is another command sent to the encoder to reset it's internal reference

## Comms init procedure for the two MODBUSes
- ExDriver will start with bus 1, firing a discovery for 1-7 MODBUS addresses three times to ensure find out each encoder on the bus. If there is under three encoders it will flag a degraded operation (no skew correction or redundancy), once the encoders are found it will map them to lift, tilt and jaw, look up min and max angles and do final checks. It will repeat for the second bus.
- Once checks pass it will start the heart beat and standby for commands, relaying appropriate data to ROS2. At this point any failture is considered a fault and the ExDriver will lock out motion until the fault flag is cleared. The fault flag does not automatically clear.
- If any high priority check fails it, like redundancy or skewing being out of spec, it will disable position control and heartbeat, thus disabling all encoders. Velocity control can still be used but at discretion.
- If redundancy is lost or failed at start up, skew control will be disabled and the ExDriver will assume a degraded flag, but still give position and velocity control.

