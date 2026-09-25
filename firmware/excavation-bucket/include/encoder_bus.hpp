// encoder_bus.hpp
//
// Master side of the joint encoders: two RS485 buses (bsp::RS485_1, RS485_2)
// polled by the shared modbus core, one FreeRTOS task per bus. Each task first
// runs discovery (bus 1, then bus 2): every address gets a half-second slot in
// which it is lit with the identify LED and polled; a good reply turns the LED
// off, a failed one leaves it lit. Only encoders that answered are polled after.
// Control code
// reads the latest readings with EncoderBus::get() and sends commands with
// zero() / identify(); it never touches the buses directly.

#pragma once

#include <cstddef>
#include <cstdint>

#include "board_support.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "modbus/esp32_clock.hpp"
#include "modbus/mbus.hpp"
#include "rs485/esp32.hpp"

// Same order as hi_can::addressing::excavation::bucket::controller::encoder_group.
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
inline constexpr uint8_t kAllEncoders = (1u << kEncoderCount) - 1;

constexpr uint8_t encoder_bit(EncoderId id) { return 1u << static_cast<uint8_t>(id); }

const char* to_string(EncoderId id);

struct EncoderReading
{
    bool present = false;  // answered the discovery poll at boot; never polled otherwise

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
    // TODO: a Lost encoder is still polled at kAnglePeriodMs and each poll waits the
    // full kResponseTimeoutMs, so it takes most of the bus and slows the other
    // encoders on it to ~15-18 Hz. Back off Lost devices (e.g. 1 Hz) in MBUS until they answer.
    static constexpr uint32_t kResponseTimeoutMs = 50;
    static constexpr uint32_t kAnglePeriodMs = 20;  // per encoder; the bus may not sustain this at low baud
    static constexpr uint32_t kStatusPeriodMs = 200;
    // TODO: the encoder firmware still expects a 1 s heartbeat (kHeartbeatPeriodMs
    // 1000, 3 s timeout, encoder-board/encoder/rtos/shared_state.hpp); change it to match.
    static constexpr uint32_t kHeartbeatPeriodMs = 200;  // broadcast, once per bus

    static constexpr uint8_t kDiscoveryRounds = 3;
    static constexpr uint8_t kDiscoveryFirstSlave = 1;  // DIP 0-7
    static constexpr uint8_t kDiscoveryLastSlave = 8;
    static constexpr uint32_t kDiscoveryProbeMs = 500;  // slot per address

    // Installs both UARTs and starts the bus tasks, which discover the
    // encoders and register those in `enabled` (a mask of encoder_bit()).
    // Returns false (after logging) if anything fails; the rest of the
    // firmware keeps running without encoder data.
    bool begin(uint8_t enabled = kAllEncoders);

    // Latest reading for one encoder; false before begin().
    bool get(EncoderId id, EncoderReading* out) const;

    // True once both buses have finished discovery and started polling.
    bool ready() const;

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

    static constexpr EventBits_t discovered_bit(size_t bus_index) { return 1u << bus_index; }
    static constexpr EventBits_t kAllDiscovered = (1u << kBusCount) - 1;

    struct Slot
    {
        EncoderBus* owner;
        size_t index;
    };

    bool submit(EncoderId id, const modbus::Request& request);
    void run(Bus& bus);
    void discover(Bus& bus, size_t bus_index);
    modbus::Result transact(Bus& bus, modbus::Request request);
    void refresh_stats(const Bus& bus, size_t bus_index);

    static void bus_task_entry(void* arg);
    static void on_angle(const modbus::Response& response, void* ctx);
    static void on_status(const modbus::Response& response, void* ctx);
    static void on_state_change(uint8_t slave, modbus::DeviceState from, modbus::DeviceState to, void* ctx);
    static void on_command_done(const modbus::Response& response, void* ctx);
    static void on_probe_done(const modbus::Response& response, void* ctx);

    modbus::Esp32Clock clock_;
    Bus bus1_;
    Bus bus2_;
    Bus* buses_[kBusCount];
    Slot slots_[kEncoderCount];
    EncoderReading readings_[kEncoderCount];
    SemaphoreHandle_t mutex_ = nullptr;
    EventGroupHandle_t events_ = nullptr;
    uint8_t enabled_ = kAllEncoders;
    bool started_ = false;
};

EncoderBus& encoder_bus();
