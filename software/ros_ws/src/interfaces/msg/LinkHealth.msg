# Network link health, measured by health_check/health_monitor.
#
# Counters come from /proc/net/dev, which is cumulative since boot, so the throughput
# fields below are deltas divided by elapsed time while the error and drop counters are
# reported raw. A rising drop count alongside healthy ROS-level topic rates points at the
# link rather than at any particular node.

# Interface the figures below were read from, e.g. "wlan0".
string interface_name

# False when the interface is not listed in /proc/net/dev, in which case every numeric
# field below is zero and means nothing.
bool interface_present

float64 rx_bytes_per_sec
float64 tx_bytes_per_sec

# Cumulative since boot, not per window.
uint64 rx_errors
uint64 tx_errors
uint64 rx_dropped
uint64 tx_dropped

# Reachability probe. Disabled by default: it only runs when a host is configured, since
# an unreachable default would otherwise report a fault on every robot that does not have
# a base station up.
bool ping_enabled
string ping_host

# Whether the most recent probe got a reply. Always false while ping_enabled is false.
bool reachable

# Round-trip time of the most recent successful probe, and -1.0 when the last probe failed
# or none has completed yet. Held from the last success rather than zeroed, so a consumer
# can distinguish "no reply" from "replied instantly".
float64 rtt_ms
