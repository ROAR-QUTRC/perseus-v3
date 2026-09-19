// encoder_task.hpp
//
// Core 1: owns the AS5600 and its I2C bus exclusively. Reads the angle on
// a steady cycle, computes velocity direction from consecutive readings,
// publishes an EncoderSample for the other core, and drains any pending
// EncoderCommand (currently just zero-reset) between reads.

#pragma once

// FreeRTOS task entry point. `parameter` must be a `SharedState*` (see
// shared_state.hpp) that outlives the task -- main() owns it.
void encoder_task(void* parameter);
