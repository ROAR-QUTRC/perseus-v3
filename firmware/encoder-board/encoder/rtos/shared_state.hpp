// shared_state.hpp
//
// Data shared between cores. encoder_task owns the AS5600; the other core gets
// its samples, and asks for a zero, through FreeRTOS queues.

#pragma once

#include "FreeRTOS.h"
#include "as5600.hpp"
#include "queue.h"

// Latest reading, published by encoder_task into a length-1 overwrite queue
// and read with xQueuePeek() (never consumed).
struct EncoderSample
{
    bool magnet_detected = false;
    bool valid = false;  // false if the last I2C read failed; reading is meaningless then
    As5600::Reading reading{};
    int velocity_sign = 0;  // -1/0/1, computed by encoder_task
    bool zero_saved = false;  // the zero offset in use is stored in flash
};

// Posted by comms_task, consumed by encoder_task (the only task that may call As5600::zero()).
enum class EncoderCommand
{
    kZero,
};

// Snapshot published by comms_task after each poll, read by led_task.
struct HeartbeatState
{
    bool heartbeat_ever_seen = false;
    bool master_alive = false;
    uint32_t last_heartbeat_ms = 0;
};

// Heartbeat cadence shared by comms_task (alive tracking) and led_task (blink timing).
inline constexpr uint32_t kHeartbeatPeriodMs = 1000;
inline constexpr uint32_t kHeartbeatTimeoutMs = 3 * kHeartbeatPeriodMs;  // a few missed beats -> "lost"

// Created in main() before the scheduler starts; tasks get a pointer.
// device_id and the boot zero are set once at boot and never written again, so
// either core may read them without synchronization.
struct SharedState
{
    QueueHandle_t latest_sample;     // EncoderSample,  length 1, xQueueOverwrite/xQueuePeek
    QueueHandle_t encoder_commands;  // EncoderCommand, length 4, xQueueSend/xQueueReceive
    QueueHandle_t heartbeat_state;   // HeartbeatState, length 1, xQueueOverwrite/xQueuePeek
    // bool, length 1. Published immediately by the discovery write handler (not
    // batched with heartbeat_state) so discovery on/off takes effect right away.
    QueueHandle_t discovery_active;
    uint8_t device_id;
    uint16_t boot_zero_offset;  // loaded from flash by main(); 0 if none was saved
    bool boot_zero_saved;
};
