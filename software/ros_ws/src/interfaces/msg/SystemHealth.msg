# Aggregate health of the sensing and estimation stack, published by
# health_check/health_monitor.
#
# One message per publish cycle carrying every monitored topic, so a consumer gets a
# consistent snapshot rather than having to correlate per-topic messages that arrived at
# different times.

# Every monitored topic is OK, and the link is reachable if a probe is configured.
uint8 STATUS_OK=0
# At least one topic is SLOW, or the link is showing errors or drops. The stack is still
# running and usable, but something is not keeping up.
uint8 STATUS_DEGRADED=1
# At least one topic is STALE or has no publisher, or a configured ping host stopped
# answering. Something the stack depends on is not there.
uint8 STATUS_ERROR=2

std_msgs/Header header

# Worst status across all topics and the link, so a consumer can gate on a single field
# without walking the array.
uint8 overall_status

# In the order the topics were configured, not the order messages arrived.
TopicHealth[] topics

LinkHealth link
