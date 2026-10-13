# Reachability of one rover device from this machine, measured by link_monitor with ICMP
# ping. Part of LinkHealth.
#
# A figure the monitor could not measure is -1, so a consumer can tell "no reading" from a
# real zero.

# Replies are arriving and the round trip and loss are inside the configured limits.
uint8 STATUS_OK=0
# Replies are arriving, but the round trip is slow or too many probes are being lost.
uint8 STATUS_DEGRADED=1
# Several probes in a row went unanswered, or ping could not run at all.
uint8 STATUS_DOWN=2

# Device name from config/network_devices.toml, e.g. medium-brain.
string name
string ip
uint8 status

# Mean round trip of the replies in the trailing window, in milliseconds. -1 if none arrived.
float32 rtt_ms

# Standard deviation of the round trips in the same window, in milliseconds. -1 if fewer
# than two replies arrived.
float32 jitter_ms

# Share of probes in the trailing window that went unanswered, 0 to 100.
float32 loss_percent
