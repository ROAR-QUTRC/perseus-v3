// comms_task.hpp
//
// Core 0: owns the RS485 transport and the Modbus RTU slave. Services one
// request per loop iteration and answers register reads/writes from the
// latest EncoderSample published by encoder_task, rather than touching
// the AS5600 directly.

#pragma once

// FreeRTOS task entry point. `parameter` must be a `SharedState*` (see
// shared_state.hpp) that outlives the task -- main() owns it.
void comms_task(void* parameter);
