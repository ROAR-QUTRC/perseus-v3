#pragma once

#include <cstdint>

#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "modbus/clock.hpp"

namespace modbus
{
    // modbus::Clock for the ESP32, backed by the 64-bit microsecond timer.
    class Esp32Clock : public Clock
    {
    public:
        uint32_t now_ms() override { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }
        uint32_t now_us() override { return static_cast<uint32_t>(esp_timer_get_time()); }

        // Short waits spin; long ones (low baud rates) sleep, and may overshoot by
        // a tick, which is harmless for an inter-frame gap.
        void delay_us(uint32_t us) override
        {
            if (us >= 2000)
                vTaskDelay(pdMS_TO_TICKS((us + 999) / 1000) + 1);
            else
                esp_rom_delay_us(us);
        }
    };
}  // namespace modbus
