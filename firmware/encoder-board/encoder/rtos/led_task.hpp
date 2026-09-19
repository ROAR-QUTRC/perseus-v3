// led_task.hpp
//
// Core 0: drives the status LED from the latest EncoderSample (for
// velocity) and the comms task's Modbus heartbeat state. Deliberately
// runs at a *higher* FreeRTOS priority than comms_task on this same
// core -- see the note in comms_task.cpp / rs485_transport.hpp:
// Rs485Transport::receive() busy-waits rather than blocking
// cooperatively, so if this task weren't strictly higher priority, a
// lower-or-equal-priority comms_task could starve it of CPU time
// entirely while polling. Preemptive scheduling means the higher
// priority here is enough for correctness without changing that
// busy-wait -- the tick interrupt forces the switch regardless of
// whether comms_task ever yields.

#pragma once

// FreeRTOS task entry point. `parameter` must be a `SharedState*` (see
// shared_state.hpp) that outlives the task -- main() owns it. Reads
// velocity from shared->latest_sample and heartbeat state from
// shared->heartbeat_state; never touches As5600 or ModbusRtu directly.
void led_task(void* parameter);
