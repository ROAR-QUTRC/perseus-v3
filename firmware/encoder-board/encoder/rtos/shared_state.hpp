// shared_state.hpp
//
// Cross-core data: core 1 (encoder_task) owns the AS5600 and is the only
// thing allowed to touch it (see the "not thread/core safe" note in
// as5600.hpp); core 0 (comms_task, led_task) needs its readings and needs
// a way to ask for a zero-reset without calling into it directly. FreeRTOS
// queues carry both directions safely under the SMP kernel.

#pragma once

#include "FreeRTOS.h"
#include "as5600.hpp"
#include "queue.h"

// Published by encoder_task after every read; comms_task and led_task
// read it with xQueuePeek() (never consume) since it's a length-1
// "overwrite" queue that always holds just the latest sample.
struct EncoderSample
{
    bool magnet_detected = false;
    bool valid = false;  // false if the last I2C read failed; reading is meaningless then
    As5600::Reading reading{};
    int velocity_sign = 0;  // -1/0/1, computed by encoder_task from consecutive readings
};

// Posted by comms_task when a Modbus write targets kRegZeroCommand;
// consumed by encoder_task, since it's the only task allowed to call
// As5600::zero().
enum class EncoderCommand
{
    kZero,
};

// Published by comms_task after every modbus.poll() cycle; read by
// led_task. Keeps ModbusRtu fully private to comms_task -- led_task
// never touches it directly, just this mirrored snapshot, the same
// publish-latest-state pattern as EncoderSample above.
struct HeartbeatState
{
    bool heartbeat_ever_seen = false;
    bool master_alive = false;
    uint32_t last_heartbeat_ms = 0;
};

// The heartbeat cadence both comms_task (tracks master_alive) and
// led_task (derives its whole blink/chase timing from it) need to agree
// on -- see status_led.hpp for how the rest of the visual language scales
// off this one number.
inline constexpr uint32_t kHeartbeatPeriodMs = 1000;
inline constexpr uint32_t kHeartbeatTimeoutMs = 3 * kHeartbeatPeriodMs;  // a few missed beats -> "lost"

// Created once in main() before the scheduler starts, then handed by
// pointer to every task that needs it. device_id is read once at boot
// (see board_id.hpp) and never written again, so it's safe to read from
// either core without synchronization.
struct SharedState
{
    QueueHandle_t latest_sample;     // EncoderSample,  length 1, xQueueOverwrite/xQueuePeek
    QueueHandle_t encoder_commands;  // EncoderCommand, length 4, xQueueSend/xQueueReceive
    QueueHandle_t heartbeat_state;   // HeartbeatState, length 1, xQueueOverwrite/xQueuePeek
    // bool, length 1, xQueueOverwrite/xQueuePeek. Published immediately by
    // comms_task's kRegDiscovery write handler (not batched with
    // heartbeat_state, which only updates once per poll() cycle) since a
    // discovery on/off command should take effect right away.
    QueueHandle_t discovery_active;
    uint8_t device_id;
};
