// encoder_bus.hpp
//
// Master side of the joint encoders: two RS485 buses (bsp::RS485_1, RS485_2)
// polled by the shared modbus core, one FreeRTOS task per bus. Control code
// reads the latest readings with EncoderBus::get() and sends commands with
// zero() / identify(); it never touches the buses directly.

#pragma once

#include <cstddef>
#include <cstdint>

#include "board_support.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "modbus/esp32_clock.hpp"
#include "modbus/mbus.hpp"
#include "rs485/esp32.hpp"

// Same order as hi_can::addressing::excavation::bucket::controller_board::encoder_group.
enum class EncoderId : uint8_t
{
    LiftLeft,
    LiftRight,
    TiltLeft,
    TiltRight,
    JawsLeft,
    JawsRight,
};

inline constexpr size_t kEncoderCount = 6;

const char* to_string(EncoderId id);

struct EncoderReading
{
    // Set by a good angle read. Cleared when the encoder answers with an
    // exception (no magnet, or a failed I2C read on the board). Link failures
    // leave the last value in place: check `link` and angle_age_ms().
    bool angle_valid = false;
    uint16_t raw_counts = 0;  // 0-4095, after the board's zero offset
    float degrees = 0.0f;
    uint32_t angle_timestamp_ms = 0;

    bool status_valid = false;
    bool magnet_detected = false;
    bool master_alive = false;

    modbus::DeviceState link = modbus::DeviceState::Unknown;
    modbus::DeviceStats stats{};

    uint32_t angle_age_ms(uint32_t now_ms) const
    {
        return angle_valid ? now_ms - angle_timestamp_ms : UINT32_MAX;
    }
};

class EncoderBus
{
public:
    static constexpr uint32_t kBaudHz = 115200;  // must match MODBUS_BAUD_HZ in the encoder's rtos/comms_task.cpp
    static constexpr uint32_t kResponseTimeoutMs = 50;
    static constexpr uint32_t kAnglePeriodMs = 20;  // per encoder; the bus may not sustain this at low baud
    static constexpr uint32_t kStatusPeriodMs = 200;
    static constexpr uint32_t kHeartbeatPeriodMs = 1000;  // broadcast, once per bus

    // Installs both UARTs, registers the six encoders and starts the bus
    // tasks. Returns false (after logging) if anything fails; the rest of the
    // firmware keeps running without encoder data.
    bool begin();

    // Latest reading for one encoder; false before begin().
    bool get(EncoderId id, EncoderReading* out) const;

    // Clock the reading timestamps come from, for computing ages.
    uint32_t now_ms() { return clock_.now_ms(); }

    // Queued for the bus task. Return false if the queue is full or not started.
    bool zero(EncoderId id);
    bool identify(EncoderId id, bool on);

private:
    friend EncoderBus& encoder_bus();
    EncoderBus();

    static constexpr size_t kBusCount = 2;
    static constexpr size_t kDevicesPerBus = 3;
    static constexpr size_t kRequestsPerBus = 8;

    struct Bus
    {
        Bus(uart_port_t uart, const bsp::rs485_pins_t& pins, modbus::Clock& clock, EncoderBus* owner)
            : port(uart, pins.tx, pins.rx, pins.dir, kBaudHz),
              mbus(port, clock, kResponseTimeoutMs),
              owner(owner)
        {
        }

        rs485::Esp32Port port;
        modbus::MBUS<kDevicesPerBus, kRequestsPerBus> mbus;
        QueueHandle_t commands = nullptr;  // modbus::Request from other tasks
        EncoderBus* owner;
    };

    struct Slot
    {
        EncoderBus* owner;
        size_t index;
    };

    bool submit(EncoderId id, const modbus::Request& request);
    void run(Bus& bus);
    void refresh_stats(const Bus& bus, size_t bus_index);

    static void bus_task_entry(void* arg);
    static void on_angle(const modbus::Response& response, void* ctx);
    static void on_status(const modbus::Response& response, void* ctx);
    static void on_state_change(uint8_t slave, modbus::DeviceState from, modbus::DeviceState to, void* ctx);
    static void on_command_done(const modbus::Response& response, void* ctx);

    modbus::Esp32Clock clock_;
    Bus bus1_;
    Bus bus2_;
    Bus* buses_[kBusCount];
    Slot slots_[kEncoderCount];
    EncoderReading readings_[kEncoderCount];
    SemaphoreHandle_t mutex_ = nullptr;
    bool started_ = false;
};

EncoderBus& encoder_bus();
