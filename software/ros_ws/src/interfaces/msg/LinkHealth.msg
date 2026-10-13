# State of the radio link between this machine and the rover, published by link_monitor.
#
# Throughput is read from the byte counters of whichever interface the kernel routes rover
# traffic over, so it counts everything on that interface rather than only ROS traffic. The
# interface is re-detected every few seconds, so docking onto ethernet or dropping to wifi
# is followed without a restart.

uint8 STATUS_OK=0
# Reachable, but a device is degraded or the link is close to its capacity.
uint8 STATUS_DEGRADED=1
# No route to the rover, or every device is unreachable.
uint8 STATUS_ERROR=2

std_msgs/Header header

# Worst of the link and every device, so a consumer can gate on one field.
uint8 overall_status

# Interface the rover is being reached over, or empty if there is no route.
string interface

# Throughput of that interface since the previous message, in bytes per second.
# -1 on the first message after the interface changes.
float64 rx_bytes_per_sec
float64 tx_bytes_per_sec

# Negotiated speed of the interface in megabits per second, as the kernel reports it.
# -1 when it does not know, which is usual for wifi.
float32 link_speed_mbps

# In the order the devices were configured.
DeviceLink[] devices
