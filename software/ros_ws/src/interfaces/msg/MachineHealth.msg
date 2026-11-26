# Host resource usage of one computer, published by system_monitor.
#
# Each computer in the system runs its own monitor under a namespace named after the
# machine, so a consumer finds every machine by listing the /<device_name>/system_health
# topics rather than being told the machine names up front.
#
# A field the monitor could not measure is -1 (or 0 for the byte counts), so a consumer
# can tell "no reading" from a real zero.

std_msgs/Header header

# Name of the machine, the same string as the topic namespace.
string device_name

# Share of CPU time spent non-idle across all cores since the previous message, 0 to
# 100. -1 on the first message, which has no earlier sample to compare against.
float32 cpu_percent

# Share of physical memory in use, 0 to 100. Memory the kernel can reclaim on demand
# (page cache) counts as free, matching MemAvailable in /proc/meminfo.
float32 memory_percent

# Memory in use and total physical memory, in bytes.
uint64 memory_used_bytes
uint64 memory_total_bytes

# Hottest temperature sensor on the machine, in degrees Celsius. -1 if the machine
# exposes none (typical in a VM).
float32 temperature_c
