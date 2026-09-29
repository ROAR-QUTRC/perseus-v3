#include "bank_control_task.hpp"

#include "freertos/task.h"

namespace
{
    // = EncoderBus::kAnglePeriodMs; no point ticking faster than encoder data
    // actually refreshes.
    constexpr uint32_t kPeriodMs = 20;
    // Below EncoderBus's RS485 tasks (priority 5, latency-sensitive UART I/O
    // position control depends on), above the Arduino loop task (priority 1,
    // CAN handling).
    constexpr UBaseType_t kTaskPriority = 4;
    constexpr BaseType_t kTaskCore = 0;  // same core as EncoderBus
    constexpr uint32_t kStackBytes = 4096;

    std::array<MotorBank*, kBankCount> g_banks{};
    TaskHandle_t g_task = nullptr;

    void task_entry(void*)
    {
        for (;;)
        {
            const uint32_t now_ms = encoder_bus().now_ms();
            for (MotorBank* bank : g_banks)
                if (bank)
                    bank->control_tick(now_ms);
            // Returns early when wake_bank_control_task() is called.
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kPeriodMs));
        }
    }
}  // namespace

bool start_bank_control_task(const std::array<MotorBank*, kBankCount>& banks)
{
    g_banks = banks;
    return xTaskCreatePinnedToCore(&task_entry, "bank_control", kStackBytes, nullptr,
                                   kTaskPriority, &g_task, kTaskCore) == pdPASS;
}

void wake_bank_control_task()
{
    if (g_task)
        xTaskNotifyGive(g_task);
}
