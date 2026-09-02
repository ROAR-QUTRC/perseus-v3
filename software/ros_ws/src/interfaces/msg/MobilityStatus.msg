# Result of comparing commanded velocity on /cmd_vel_out against velocity measured on
# /odometry/filtered, produced by watchdog/mobility_watchdog.
#
# The filtered odometry has no wheel-encoder input (see autonomy_bringup/config/
# ekf_config.yaml) -- it is derived from FAST-LIO's LiDAR-inertial pose and the IMU gyro --
# so a sustained gap between command and measurement reflects the wheels genuinely failing
# to move the robot, such as when they lose traction in sand, rather than two wheel-derived
# signals agreeing with each other for the wrong reason.

std_msgs/Header header

# Whether the linear or angular channel has enough sustained command history to report a
# fresh assessment this cycle. False while /cmd_vel_out is stale or the robot is
# essentially idle.
#
# This is the OR of the two channels, so it cannot say which one the assessment came from:
# driving straight it reads true even though the angular channel is idle. Consumers that
# care per channel should read the efficiency fields below, which carry their own
# not-evaluating sentinel.
bool is_evaluating

# Whether sustained slip was detected on the linear or angular channel this cycle.
# Always false while is_evaluating is false.
bool is_slipping

# Mean slip ratio over the detection window for forward speed: 0 means commanded speed
# was fully achieved, 1 means no measured forward motion at all despite the command. Not
# clamped, so it can go negative when the robot exceeds the commanded speed (e.g. rolling
# downhill). Reset to 0 while that channel is not being evaluated, so a 0 here is not by
# itself evidence that the robot is tracking its command.
float64 linear_slip_ratio

# Same ratio computed from yaw rate instead of forward speed. Generally more trustworthy
# than the linear ratio: the measured value here comes straight from the IMU gyro, whereas
# the measured linear speed is inferred by the EKF from LiDAR pose differencing.
float64 angular_slip_ratio

# Percentage of the commanded motion actually achieved on each channel: 100 means the
# commanded value was fully reached (no slip), 0 means no measured motion at all despite the
# command. Derived from the slip ratios above and clamped to 0..100, so a robot overrunning
# its command (rolling downhill, negative slip ratio) reads 100 rather than more.
#
# -1 when that channel is not being evaluated, because its commanded magnitude is below
# min_linear_speed_mps / min_angular_speed_rps and there is nothing to measure. Unlike the
# slip ratios, which reset to 0 and so read as perfect tracking when idle, these separate
# "achieved everything commanded" from "nothing was commanded" -- a display that ignores the
# -1 would report a confident 100% for a wheel that is not turning. Same
# negative-means-not-measured convention as TopicHealth/age_sec.
float64 linear_efficiency
float64 angular_efficiency

# Signed commanded and measured values actually compared this cycle, for a consumer that
# wants to apply its own thresholding rather than trusting is_slipping.
float64 commanded_linear_velocity
float64 measured_linear_velocity
float64 commanded_angular_velocity
float64 measured_angular_velocity
