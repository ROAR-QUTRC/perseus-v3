# mission_bt_server's mission state, on /mission/status. Latched (transient local,
# depth 1), so a panel that starts late still sees the current state at once.
#
# Published whenever any field below changes, plus once when a mission ends.

std_msgs/Header header

uint8 STATE_IDLE = 0       # nothing has run since the server started
uint8 STATE_RUNNING = 1
uint8 STATE_SUCCEEDED = 2
uint8 STATE_FAILED = 3
uint8 STATE_STOPPED = 4    # cancelled through /mission/stop
uint8 state

# Echo of the running (or last) mission's StartMission request fields.
uint8 mode
uint8 task
uint32 cycles_target
uint32 cycles_completed

# What the tree is doing right now, e.g. "to_excavation", "excavating",
# "to_construction", "depositing", "done". Empty while idle.
string phase

# Human-readable detail: why it failed, or a short summary once finished.
string message
