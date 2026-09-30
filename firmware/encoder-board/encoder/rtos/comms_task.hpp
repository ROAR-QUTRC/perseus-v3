// comms_task.hpp
//
// Core 0: owns the RS485 port and the Modbus RTU slave, answering register
// requests from the latest EncoderSample.

#pragma once

// `parameter` must be a SharedState* that outlives the task.
void comms_task(void* parameter);
