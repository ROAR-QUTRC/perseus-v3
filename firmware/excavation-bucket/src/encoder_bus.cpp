// encoder_bus.cpp

#include "encoder_bus.hpp"

#include <cmath>

#include "esp_log.h"
#include "excavation_config.hpp"
#include "lock.hpp"
#include "modbus/profiles/encoder.hpp"

namespace enc = modbus::profiles::encoder;

namespace
{
    const char* const TAG = "encoder_bus";

    struct Placement
    {
        uint8_t bus;  // index: ENCODER_BUS - 1
        uint8_t slave;
    };

    // Which bus and Modbus address each encoder is at: set per driver in excavation_config.hpp.
    constexpr Placement placement_of(EncoderId id)
    {
        switch (id)
        {
        case LIFT::DRIVER_A::ENCODER_ID:
            return {LIFT::DRIVER_A::ENCODER_BUS - 1, LIFT::DRIVER_A::ENCODER_SLAVE};
        case LIFT::DRIVER_B::ENCODER_ID:
            return {LIFT::DRIVER_B::ENCODER_BUS - 1, LIFT::DRIVER_B::ENCODER_SLAVE};
        case TILT::DRIVER_A::ENCODER_ID:
            return {TILT::DRIVER_A::ENCODER_BUS - 1, TILT::DRIVER_A::ENCODER_SLAVE};
        case TILT::DRIVER_B::ENCODER_ID:
            return {TILT::DRIVER_B::ENCODER_BUS - 1, TILT::DRIVER_B::ENCODER_SLAVE};
        case JAWS::DRIVER_A::ENCODER_ID:
            return {JAWS::DRIVER_A::ENCODER_BUS - 1, JAWS::DRIVER_A::ENCODER_SLAVE};
        case JAWS::DRIVER_B::ENCODER_ID:
            return {JAWS::DRIVER_B::ENCODER_BUS - 1, JAWS::DRIVER_B::ENCODER_SLAVE};
        }
        return {0xFF, 0};
    }

    // Indexed by EncoderId.
    constexpr Placement kPlacement[kEncoderCount] = {
        placement_of(EncoderId::LiftLeft),
        placement_of(EncoderId::LiftRight),
        placement_of(EncoderId::TiltLeft),
        placement_of(EncoderId::TiltRight),
        placement_of(EncoderId::JawsLeft),
        placement_of(EncoderId::JawsRight),
    };

    constexpr uint8_t kMaxSlave = 8;  // the encoder's 3-bit DIP value + 1

    constexpr bool placements_valid(size_t bus_count, size_t devices_per_bus)
    {
        for (size_t i = 0; i < kEncoderCount; ++i)
        {
            if (kPlacement[i].bus >= bus_count || kPlacement[i].slave < 1 || kPlacement[i].slave > kMaxSlave)
                return false;
            size_t on_this_bus = 0;
            for (size_t j = 0; j < kEncoderCount; ++j)
            {
                if (kPlacement[j].bus != kPlacement[i].bus)
                    continue;
                ++on_this_bus;
                if (j != i && kPlacement[j].slave == kPlacement[i].slave)
                    return false;  // two encoders at one address
            }
            if (on_this_bus > devices_per_bus)
                return false;
        }
        return true;
    }

    const char* const kNames[kEncoderCount] = {
        "lift_left",
        "lift_right",
        "tilt_left",
        "tilt_right",
        "jaws_left",
        "jaws_right",
    };

    constexpr uint32_t kBusTaskStackBytes = 4096;
    constexpr UBaseType_t kBusTaskPriority = 5;  // above the Arduino loop task (1)
    constexpr BaseType_t kBusTaskCore = 0;       // loop() and the motor code run on core 1
    constexpr int kCommandRetries = 2;
}  // namespace

const char* to_string(EncoderId id)
{
    const size_t index = static_cast<size_t>(id);
    return index < kEncoderCount ? kNames[index] : "unknown";
}

EncoderBus& encoder_bus()
{
    static EncoderBus instance;
    return instance;
}

EncoderBus::EncoderBus()
    : bus1_(UART_NUM_1, bsp::RS485_1, clock_, this),
      bus2_(UART_NUM_2, bsp::RS485_2, clock_, this),
      buses_{&bus1_, &bus2_}
{
    static_assert(placements_valid(kBusCount, kDevicesPerBus),
                  "excavation_config.hpp: each ENCODER_BUS must be 1 or 2, each ENCODER_SLAVE 1-8, "
                  "with no two encoders at the same address on one bus and at most kDevicesPerBus per bus");
}

bool EncoderBus::begin(uint8_t enabled)
{
    if (started_)
        return true;

    enabled_ = enabled;
    mutex_ = xSemaphoreCreateMutex();
    events_ = xEventGroupCreate();
    if (mutex_ == nullptr || events_ == nullptr)
    {
        ESP_LOGE(TAG, "failed to create mutex or event group");
        return false;
    }

    for (Bus* bus : buses_)
    {
        bus->commands = xQueueCreate(kRequestsPerBus, sizeof(modbus::Request));
        if (bus->commands == nullptr || !bus->port.init())
        {
            ESP_LOGE(TAG, "failed to start RS485 bus");
            return false;
        }
    }

    for (size_t i = 0; i < kEncoderCount; ++i)
        slots_[i] = {this, i};

    static const char* const kTaskNames[kBusCount] = {"rs485_bus1", "rs485_bus2"};
    for (size_t b = 0; b < kBusCount; ++b)
    {
        if (xTaskCreatePinnedToCore(&EncoderBus::bus_task_entry, kTaskNames[b], kBusTaskStackBytes, buses_[b],
                                    kBusTaskPriority, nullptr, kBusTaskCore) != pdPASS)
        {
            ESP_LOGE(TAG, "failed to start %s", kTaskNames[b]);
            return false;
        }
    }

    started_ = true;
    return true;
}

bool EncoderBus::get(EncoderId id, EncoderReading* out) const
{
    const size_t index = static_cast<size_t>(id);
    if (!started_ || index >= kEncoderCount)
        return false;

    Lock lock(mutex_);
    *out = readings_[index];
    return true;
}

bool EncoderBus::ready() const
{
    return events_ != nullptr && (xEventGroupGetBits(events_) & kAllDiscovered) == kAllDiscovered;
}

bool EncoderBus::zero(EncoderId id)
{
    const size_t index = static_cast<size_t>(id);
    if (index >= kEncoderCount)
        return false;
    return submit(id, enc::zero_request(kPlacement[index].slave, &EncoderBus::on_command_done, &slots_[index]));
}

bool EncoderBus::identify(EncoderId id, bool on)
{
    const size_t index = static_cast<size_t>(id);
    if (index >= kEncoderCount)
        return false;
    return submit(id, enc::discovery_request(kPlacement[index].slave, on, &EncoderBus::on_command_done,
                                             &slots_[index]));
}

bool EncoderBus::submit(EncoderId id, const modbus::Request& request)
{
    if (!started_)
        return false;

    modbus::Request queued = request;
    queued.retries = kCommandRetries;
    Bus& bus = *buses_[kPlacement[static_cast<size_t>(id)].bus];
    return xQueueSend(bus.commands, &queued, 0) == pdTRUE;
}

void EncoderBus::bus_task_entry(void* arg)
{
    auto* bus = static_cast<Bus*>(arg);
    bus->owner->run(*bus);
}

void EncoderBus::run(Bus& bus)
{
    size_t bus_index = 0;
    for (size_t b = 0; b < kBusCount; ++b)
        if (buses_[b] == &bus)
            bus_index = b;

    // One bus at a time, so the identify LEDs sweep bus 1 then bus 2.
    if (bus_index > 0)
        xEventGroupWaitBits(events_, discovered_bit(bus_index - 1), pdFALSE, pdTRUE, portMAX_DELAY);
    discover(bus, bus_index);
    xEventGroupSetBits(events_, discovered_bit(bus_index));

    uint32_t last_heartbeat_ms = clock_.now_ms() - kHeartbeatPeriodMs;  // first one goes out immediately
    uint32_t last_reprobe_ms = clock_.now_ms();
    size_t reprobe_cursor = 0;

    for (;;)
    {
        modbus::Request request;
        while (xQueueReceive(bus.commands, &request, 0) == pdTRUE)
            bus.mbus.submit(request);

        const uint32_t now = clock_.now_ms();
        if (now - last_heartbeat_ms >= kHeartbeatPeriodMs && bus.mbus.submit(enc::heartbeat_request()))
            last_heartbeat_ms = now;

        if (now - last_reprobe_ms >= kReprobePeriodMs)
        {
            last_reprobe_ms = now;
            reprobe_next_missing(bus, bus_index, &reprobe_cursor);
        }

        if (bus.mbus.step())
        {
            refresh_stats(bus, bus_index);
            register_answered(bus, bus_index);
        }
        else
        {
            vTaskDelay(1);
        }
    }
}

// Queues one angle poll for the next encoder on this bus that hasn't been
// found, so one plugged in (or re-addressed) after boot still joins.
void EncoderBus::reprobe_next_missing(Bus& bus, size_t bus_index, size_t* cursor)
{
    for (size_t n = 0; n < kEncoderCount; ++n)
    {
        const size_t i = (*cursor + n) % kEncoderCount;
        if (kPlacement[i].bus != bus_index || !(enabled_ & (1u << i)))
            continue;
        {
            Lock lock(mutex_);
            if (readings_[i].present)
                continue;
        }

        modbus::Request poll;
        poll.slave = kPlacement[i].slave;
        poll.kind = modbus::Request::Kind::ReadRegs;
        poll.reg = enc::kRegAngleRaw;
        poll.value_or_count = 2;
        poll.on_done = &EncoderBus::on_reprobe_done;
        poll.ctx = &slots_[i];
        bus.mbus.submit(poll);
        *cursor = i + 1;
        return;
    }
}

// Registration changes the scheduler's device list, so it happens here, after
// step() has returned, rather than inside the reply callback.
void EncoderBus::register_answered(Bus& bus, size_t bus_index)
{
    for (size_t i = 0; i < kEncoderCount; ++i)
    {
        if (kPlacement[i].bus != bus_index || !slots_[i].answered)
            continue;
        slots_[i].answered = false;
        if (register_encoder(bus, i))
            ESP_LOGI(TAG, "bus %u: %s (slave %u) found after boot", static_cast<unsigned>(bus_index + 1), kNames[i],
                     kPlacement[i].slave);
    }
}

// Marks the encoder present and starts polling it. False if it is disabled
// or the scheduler has no room.
bool EncoderBus::register_encoder(Bus& bus, size_t index)
{
    {
        Lock lock(mutex_);
        readings_[index].present = true;
    }

    if (!(enabled_ & (1u << index)))
        return false;

    enc::DeviceConfig config;
    config.slave = kPlacement[index].slave;
    config.angle_period_ms = kAnglePeriodMs;
    config.status_period_ms = kStatusPeriodMs;
    config.on_angle = &EncoderBus::on_angle;
    config.on_status = &EncoderBus::on_status;
    config.on_state_change = &EncoderBus::on_state_change;
    config.ctx = &slots_[index];

    if (bus.mbus.add_device(enc::make_device(config)))
        return true;
    ESP_LOGE(TAG, "failed to register %s", kNames[index]);
    return false;
}

// Runs before any device is registered, so each step() executes exactly the
// request just submitted.
void EncoderBus::discover(Bus& bus, size_t bus_index)
{
    uint8_t good[kEncoderCount] = {};     // valid angle read
    uint8_t faults[kEncoderCount] = {};   // answered with an exception, e.g. no magnet
    uint8_t garbled[kEncoderCount] = {};  // something came back, but not a valid reply
    const unsigned bus_no = static_cast<unsigned>(bus_index + 1);

    ESP_LOGI(TAG, "bus %u: discovering, %u rounds", bus_no, kDiscoveryRounds);

    // Each round probes this bus's encoders in EncoderId order.
    for (uint8_t round = 0; round < kDiscoveryRounds; ++round)
    {
        for (size_t i = 0; i < kEncoderCount; ++i)
        {
            if (kPlacement[i].bus != bus_index)
                continue;

            const uint8_t slave = kPlacement[i].slave;
            TickType_t wake = xTaskGetTickCount();

            transact(bus, enc::discovery_request(slave, true));
            vTaskDelayUntil(&wake, pdMS_TO_TICKS(kDiscoveryBlipMs));
            transact(bus, enc::discovery_request(slave, false));

            modbus::Request poll;
            poll.slave = slave;
            poll.kind = modbus::Request::Kind::ReadRegs;
            poll.reg = enc::kRegAngleRaw;
            poll.value_or_count = 2;
            const modbus::Result result = transact(bus, poll);

            if (result == modbus::Result::Ok)
                ++good[i];
            else if (result == modbus::Result::ExceptionReply)
                ++faults[i];
            else if (result != modbus::Result::Timeout)
                ++garbled[i];

            vTaskDelayUntil(&wake, pdMS_TO_TICKS(kDiscoveryProbeMs - kDiscoveryBlipMs));
        }
    }

    size_t registered = 0;
    for (size_t i = 0; i < kEncoderCount; ++i)
    {
        if (kPlacement[i].bus != bus_index)
            continue;

        const uint8_t slave = kPlacement[i].slave;
        const char* name = kNames[i];

        if (garbled[i] > 0)
            ESP_LOGW(TAG, "bus %u: %s (slave %u) replied %u/%u times but the reply was unreadable (CRC/frame error)",
                     bus_no, name, slave, garbled[i], kDiscoveryRounds);
        if (faults[i] > 0)
            ESP_LOGW(TAG, "bus %u: %s (slave %u) answered %u/%u times with a fault (no magnet?)", bus_no, name, slave,
                     faults[i], kDiscoveryRounds);

        if (good[i] == 0 && faults[i] == 0)
        {
            ESP_LOGW(TAG, "bus %u: %s (slave %u) missing", bus_no, name, slave);
            continue;
        }
        if (good[i] == kDiscoveryRounds)
            ESP_LOGI(TAG, "bus %u: %s (slave %u) found", bus_no, name, slave);
        else
            ESP_LOGW(TAG, "bus %u: %s (slave %u) flaky, good %u/%u", bus_no, name, slave, good[i],
                     kDiscoveryRounds);

        if (register_encoder(bus, i))
            ++registered;
    }

    if (registered < kDevicesPerBus)
        ESP_LOGW(TAG, "bus %u: degraded, polling %u/%u encoders; re-probing the rest", bus_no,
                 static_cast<unsigned>(registered), static_cast<unsigned>(kDevicesPerBus));
}

// Runs one request to completion and returns its result.
modbus::Result EncoderBus::transact(Bus& bus, modbus::Request request)
{
    modbus::Result result = modbus::Result::TransportError;
    request.on_done = &EncoderBus::on_probe_done;
    request.ctx = &result;
    if (!bus.mbus.submit(request))
        return result;
    bus.mbus.step();
    return result;
}

void EncoderBus::refresh_stats(const Bus& bus, size_t bus_index)
{
    for (size_t i = 0; i < kEncoderCount; ++i)
    {
        if (kPlacement[i].bus != bus_index)
            continue;

        modbus::DeviceStats stats;
        if (!bus.mbus.get_stats(kPlacement[i].slave, &stats))
            continue;

        Lock lock(mutex_);
        readings_[i].stats = stats;
        readings_[i].link = stats.state;
    }
}

void EncoderBus::on_angle(const modbus::Response& response, void* ctx)
{
    auto* slot = static_cast<Slot*>(ctx);
    EncoderBus& self = *slot->owner;
    EncoderReading& reading = self.readings_[slot->index];

    enc::Angle angle;
    if (enc::decode_angle(response, &angle))
    {
        Lock lock(self.mutex_);
        reading.angle_valid = true;
        reading.raw_counts = angle.raw_counts;
        // -180..180 like atan2, so an encoder a hair past its zero reads -0.1, not 359.9.
        reading.degrees = std::remainder(angle.raw_counts * (360.0f / 4096.0f), 360.0f);
        reading.angle_timestamp_ms = response.timestamp_ms;
    }
    else if (response.result == modbus::Result::ExceptionReply)
    {
        Lock lock(self.mutex_);
        reading.angle_valid = false;
    }
}

void EncoderBus::on_status(const modbus::Response& response, void* ctx)
{
    auto* slot = static_cast<Slot*>(ctx);
    EncoderBus& self = *slot->owner;

    enc::Status status;
    if (!enc::decode_status(response, &status))
        return;

    Lock lock(self.mutex_);
    EncoderReading& reading = self.readings_[slot->index];
    reading.status_valid = true;
    reading.magnet_detected = status.magnet_detected;
    reading.master_alive = status.master_alive;
}

void EncoderBus::on_state_change(uint8_t, modbus::DeviceState from, modbus::DeviceState to, void* ctx)
{
    auto* slot = static_cast<Slot*>(ctx);
    ESP_LOGW(TAG, "%s link %d -> %d", kNames[slot->index], static_cast<int>(from), static_cast<int>(to));
}

void EncoderBus::on_command_done(const modbus::Response& response, void* ctx)
{
    if (response.result == modbus::Result::Ok)
        return;

    auto* slot = static_cast<Slot*>(ctx);
    ESP_LOGW(TAG, "command to %s failed (result %d)", kNames[slot->index], static_cast<int>(response.result));
}

void EncoderBus::on_probe_done(const modbus::Response& response, void* ctx)
{
    *static_cast<modbus::Result*>(ctx) = response.result;
}

// An exception reply still means the board is there (no magnet, for one).
void EncoderBus::on_reprobe_done(const modbus::Response& response, void* ctx)
{
    if (response.result == modbus::Result::Ok || response.result == modbus::Result::ExceptionReply)
        static_cast<Slot*>(ctx)->answered = true;
}
