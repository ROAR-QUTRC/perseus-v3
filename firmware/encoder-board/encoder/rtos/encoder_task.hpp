// encoder_task.hpp
//
// Core 1: owns the AS5600 and its I2C bus. Samples the angle on a fixed cycle,
// derives velocity direction, publishes an EncoderSample, and handles pending
// EncoderCommands (zero-reset) between reads.

#pragma once

// `parameter` must be a SharedState* that outlives the task.
void encoder_task(void* parameter);
