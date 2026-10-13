// encoder.cpp
//
// Entry point: one-time board bring-up (DIP-switch ID, AS5600 DIR strap, boot
// banner), then creates the tasks and starts the scheduler.
//   encoder_task (core 1): owns the AS5600
//   comms_task   (core 0): RS485/Modbus RTU slave
//   led_task     (core 0): status LED, must outrank comms_task (see led_task.hpp)

#include <cstdio>

#include "FreeRTOS.h"
#include "comms_task.hpp"
#include "encoder_task.hpp"
#include "hardware/gpio.h"
#include "led_task.hpp"
#include "pico/stdlib.h"
#include "shared_state.hpp"
#include "task.h"

// AS5600 rotation-direction strap: high = CW, low = CCW. Externally pulled up
// (a 4th DIP position can ground it), so read as a plain input.
#ifndef AS5600_DIR_PIN
#define AS5600_DIR_PIN 8
#endif

namespace
{
    // Stack sizes are in words, not bytes.
    constexpr uint32_t kEncoderTaskStackWords = 1024;  // I2C + printf
    constexpr uint32_t kCommsTaskStackWords = 1024;    // RS485/Modbus dispatch + printf
    constexpr uint32_t kLedTaskStackWords = 512;       // just tick() + set_pixel(), no printf

    // led_task must outrank comms_task: see led_task.hpp.
    constexpr UBaseType_t kEncoderTaskPriority = tskIDLE_PRIORITY + 2;
    constexpr UBaseType_t kCommsTaskPriority = tskIDLE_PRIORITY + 1;
    constexpr UBaseType_t kLedTaskPriority = tskIDLE_PRIORITY + 3;

    constexpr UBaseType_t kCore0Affinity = (1u << 0);
    constexpr UBaseType_t kCore1Affinity = (1u << 1);

    // DIP switch (SW1), one bit per pin. A pin is shorted to GND when its
    // switch is on and floats otherwise, so it is read with the internal
    // pull-up: low means the bit is set.
    constexpr uint32_t kBoardIdPin0 = 11;  // ID_ADDR0
    constexpr uint32_t kBoardIdPin1 = 10;  // ID_ADDR1
    constexpr uint32_t kBoardIdPin2 = 9;   // ID_ADDR2

    // Returns the 3-bit device ID (0-7). Call once at boot.
    uint8_t read_board_id()
    {
        gpio_init(kBoardIdPin0);
        gpio_set_dir(kBoardIdPin0, GPIO_IN);
        gpio_pull_up(kBoardIdPin0);

        gpio_init(kBoardIdPin1);
        gpio_set_dir(kBoardIdPin1, GPIO_IN);
        gpio_pull_up(kBoardIdPin1);

        gpio_init(kBoardIdPin2);
        gpio_set_dir(kBoardIdPin2, GPIO_IN);
        gpio_pull_up(kBoardIdPin2);

        sleep_us(10);  // let the pull-ups settle before reading

        uint8_t id = 0;
        if (!gpio_get(kBoardIdPin0))
            id |= (1u << 0);
        if (!gpio_get(kBoardIdPin1))
            id |= (1u << 1);
        if (!gpio_get(kBoardIdPin2))
            id |= (1u << 2);
        return id;
    }
}  // namespace

// Called by the kernel on stack overflow. Halts: memory is already corrupted.
extern "C" void vApplicationStackOverflowHook(TaskHandle_t, char* task_name)
{
    printf("FATAL: stack overflow in task '%s'\n", task_name);
    while (true)
        tight_loop_contents();
}

int main()
{
    stdio_init_all();

    static SharedState shared{};
    shared.device_id = read_board_id();

    gpio_init(AS5600_DIR_PIN);
    gpio_set_dir(AS5600_DIR_PIN, GPIO_IN);
    bool clockwise = gpio_get(AS5600_DIR_PIN);

    // Repeated for a couple of seconds: a monitor opened just after flashing
    // would miss a one-shot print (USB CDC race).
    for (int i = 0; i < 10; ++i)
    {
        printf("device id: %u, direction: %s\n", shared.device_id, clockwise ? "CW" : "CCW");
        sleep_ms(200);
    }

    shared.latest_sample = xQueueCreate(1, sizeof(EncoderSample));
    shared.encoder_commands = xQueueCreate(4, sizeof(EncoderCommand));
    shared.heartbeat_state = xQueueCreate(1, sizeof(HeartbeatState));
    shared.discovery_active = xQueueCreate(1, sizeof(bool));
    configASSERT(shared.latest_sample && shared.encoder_commands && shared.heartbeat_state &&
                 shared.discovery_active);

    // Seed the length-1 queues so the first xQueuePeek() in comms_task/led_task succeeds.
    EncoderSample initial_sample{};
    xQueueOverwrite(shared.latest_sample, &initial_sample);
    HeartbeatState initial_heartbeat{};
    xQueueOverwrite(shared.heartbeat_state, &initial_heartbeat);
    bool initial_discovery = false;
    xQueueOverwrite(shared.discovery_active, &initial_discovery);

    BaseType_t ok = pdPASS;
    ok &= xTaskCreateAffinitySet(encoder_task, "encoder", kEncoderTaskStackWords, &shared,
                                 kEncoderTaskPriority, kCore1Affinity, nullptr);
    ok &= xTaskCreateAffinitySet(comms_task, "comms", kCommsTaskStackWords, &shared,
                                 kCommsTaskPriority, kCore0Affinity, nullptr);
    ok &= xTaskCreateAffinitySet(led_task, "led", kLedTaskStackWords, &shared, kLedTaskPriority,
                                 kCore0Affinity, nullptr);
    configASSERT(ok == pdPASS);

    vTaskStartScheduler();

    // Only reached if the scheduler failed to start (e.g. out of heap).
    printf("FATAL: vTaskStartScheduler() returned\n");
    while (true)
        tight_loop_contents();
}
