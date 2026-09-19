// led_task.cpp

#include "led_task.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include "hardware/pio.h"
#include "pico/time.h"

#include "shared_state.hpp"
#include "status_led.hpp"
#include "ws2812.hpp"

#ifndef NEOPIXEL_PIN
#define NEOPIXEL_PIN 4
#endif

void led_task(void* parameter)
{
    auto* shared = static_cast<SharedState*>(parameter);

    Ws2812 status_pixel(pio0, 0, NEOPIXEL_PIN);
    status_pixel.init();

    StatusLed status_led(kHeartbeatPeriodMs);

    for (;;)
    {
        EncoderSample sample;
        if (xQueuePeek(shared->latest_sample, &sample, 0) == pdTRUE)
        {
            status_led.update_velocity(sample.velocity_sign);
            status_led.update_magnet_detected(sample.magnet_detected);
        }

        HeartbeatState hb;
        if (xQueuePeek(shared->heartbeat_state, &hb, 0) == pdTRUE)
            status_led.update_heartbeat_state(hb.heartbeat_ever_seen, hb.master_alive, hb.last_heartbeat_ms);

        bool discovery = false;
        if (xQueuePeek(shared->discovery_active, &discovery, 0) == pdTRUE)
            status_led.update_discovery(discovery);

        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        StatusLed::Rgb color = status_led.tick(now_ms);
        status_pixel.set_pixel(color.r, color.g, color.b);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
