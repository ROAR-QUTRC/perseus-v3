// led_task.hpp
//
// Core 0: drives the status LED from the latest EncoderSample and heartbeat
// state. Runs at a higher priority than comms_task on the same core, because
// rs485::Rp2350Port::receive() busy-waits and would otherwise starve it.

#pragma once

// `parameter` must be a SharedState* that outlives the task.
void led_task(void* parameter);
