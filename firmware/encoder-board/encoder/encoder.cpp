// encoder.cpp
//
// FreeRTOS entry point. Does one-time hardware bring-up that doesn't
// belong to any single task (DIP-switch device ID, AS5600 DIR strap,
// boot-time serial banner), then creates the three tasks and starts the
// scheduler:
//   - encoder_task (core 1): owns the AS5600 exclusively
//   - comms_task   (core 0): RS485/Modbus RTU slave
//   - led_task     (core 0): status LED, strictly higher priority than
//                            comms_task on this core -- see led_task.hpp
// RP2350A silicon (revision A2), custom board, powered over USB.

#include <cstdio>

#include "FreeRTOS.h"
#include "board_id.hpp"
#include "comms_task.hpp"
#include "encoder_task.hpp"
#include "hardware/gpio.h"
#include "led_task.hpp"
#include "pico/stdlib.h"
#include "shared_state.hpp"
#include "task.h"

// Sets the AS5600's rotation-direction convention; has a light external
// pull-up on the board (plus a 4th DIP switch position that can ground it),
// so it's read as a plain input with no internal pull. High = CW, low = CCW.
#ifndef AS5600_DIR_PIN
#define AS5600_DIR_PIN 8
#endif

namespace
{
    // Stack sizes are in words (4 bytes each on this target), not bytes.
    constexpr uint32_t kEncoderTaskStackWords = 1024;  // I2C + printf
    constexpr uint32_t kCommsTaskStackWords = 1024;    // RS485/Modbus dispatch + printf
    constexpr uint32_t kLedTaskStackWords = 512;       // just tick() + set_pixel(), no printf

    // See led_task.hpp for why led_task must outrank comms_task here.
    constexpr UBaseType_t kEncoderTaskPriority = tskIDLE_PRIORITY + 2;
    constexpr UBaseType_t kCommsTaskPriority = tskIDLE_PRIORITY + 1;
    constexpr UBaseType_t kLedTaskPriority = tskIDLE_PRIORITY + 3;

    constexpr UBaseType_t kCore0Affinity = (1u << 0);
    constexpr UBaseType_t kCore1Affinity = (1u << 1);
}  // namespace

// Required by configCHECK_FOR_STACK_OVERFLOW > 0 (FreeRTOSConfig.h) -- the
// kernel calls this itself, we just need to supply it. A stack overflow is
// not recoverable (adjacent memory is already corrupted), so this halts
// rather than trying to continue.
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

    // Repeat this for a couple seconds after boot -- the USB CDC connection
    // race means a monitor opened right after flashing can easily miss a
    // one-shot print.
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

    // The length-1 "latest state" queues need a first value before
    // anything calls xQueuePeek() on them (comms_task/led_task both do,
    // immediately), otherwise that first peek just fails and they fall
    // back to their empty-state defaults for one cycle -- harmless, but
    // seeding them removes the ambiguity.
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

    // Only reached if the scheduler couldn't start (e.g. out of heap for
    // the idle/timer tasks) -- everything past this point runs as tasks.
    printf("FATAL: vTaskStartScheduler() returned\n");
    while (true)
        tight_loop_contents();
}
