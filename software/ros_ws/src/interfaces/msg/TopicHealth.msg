# Health of a single monitored topic, measured by health_check/health_monitor.
#
# Rate and bandwidth are measured by a type-agnostic generic subscription, so the monitor
# never has to link against the message type and adding a topic to the watch list is a
# config change rather than a code change. Bandwidth is therefore the size of the
# serialised CDR payload as it arrives, which is what the DDS transport actually carries;
# it excludes RTPS and UDP framing, so it reads slightly below what a packet capture on
# the wire would show.

# Everything is nominal: publishers are present and the rate is inside the tolerance band.
uint8 STATUS_OK=0
# Arriving, but measured_rate_hz has fallen below expected_rate_hz by more than the
# configured tolerance. The data is still flowing, just not fast enough.
uint8 STATUS_SLOW=1
# A publisher is advertising the topic but nothing has arrived within the stale timeout.
# Distinct from NO_PUBLISHER: the node exists and is silent, rather than absent.
uint8 STATUS_STALE=2
# Nothing is advertising the topic at all, per the ROS graph. Usually a node that failed
# to start or died, rather than a data problem.
uint8 STATUS_NO_PUBLISHER=3

string name
uint8 status

# Publishers currently advertising the topic, straight from the ROS graph. Zero means the
# producer is gone; more than one is worth investigating on topics that should have a
# single source, such as odometry.
uint32 publisher_count

# Messages per second over the trailing measurement window. Zero when nothing arrived in
# the window, which for a topic that has never published is indistinguishable from one
# that has stopped -- use status to tell those apart.
float64 measured_rate_hz

# What the topic is configured to be producing, for the consumer's own thresholding.
float64 expected_rate_hz

# Serialised payload throughput over the same window.
float64 bandwidth_bytes_per_sec

# Seconds since the most recent message arrived, and -1.0 if nothing has ever arrived
# since the monitor started. Grows without bound while a topic is silent, so it is the
# field to watch for how long an outage has lasted.
float64 age_sec

# Messages counted in the trailing window, and over the monitor's whole lifetime. The
# lifetime count keeps rising across outages and is useful for spotting a topic that is
# publishing in bursts rather than at a steady rate.
uint32 window_message_count
uint64 total_message_count
