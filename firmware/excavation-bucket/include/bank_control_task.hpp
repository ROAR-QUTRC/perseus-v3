#pragma once

#include <array>
#include <cstddef>

#include "motor_bank.hpp"

inline constexpr size_t kBankCount = 3;

// Starts the FreeRTOS task that drives control_tick() on every bank, every
// 20 ms or straight away when woken. Call once from setup(), after all
// MotorBank instances exist and after encoder_bus().begin(). Pinned to core 0
// alongside EncoderBus's RS485 tasks; loop() (CAN handling) stays on core 1.
bool start_bank_control_task(const std::array<MotorBank*, kBankCount>& banks);

// Runs the next tick now, so a new command reaches the motors without waiting
// out the period. No-op before the task has started.
void wake_bank_control_task();
