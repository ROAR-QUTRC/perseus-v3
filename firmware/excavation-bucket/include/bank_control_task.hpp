#pragma once

#include <array>
#include <cstddef>

#include "motor_bank.hpp"

inline constexpr size_t kBankCount = 3;

// Starts the fixed-period FreeRTOS task that drives control_tick() on every
// bank. Call once from setup(), after all MotorBank instances exist and
// after encoder_bus().begin(). Pinned to core 0 alongside EncoderBus's RS485
// tasks; loop() (CAN handling + open-loop SET_SPEED) stays on core 1.
bool start_bank_control_task(const std::array<MotorBank*, kBankCount>& banks);
