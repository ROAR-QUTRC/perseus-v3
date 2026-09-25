// encoder_bus.cpp

#include "encoder_bus.hpp"

#include "esp_log.h"
#include "lock.hpp"
#include "modbus/profiles/encoder.hpp"

namespace enc = modbus::profiles::encoder;

namespace
{
    const char* const TAG = "encoder_bus";

    // Left encoders on bus 1, right on bus 2, so one cable fault leaves each
    // joint with a working encoder. `slave` is the board's DIP value + 1.
    struct Placement
    {
        uint8_t bus;
        uint8_t slave;
    };

    constexpr Placement kPlacement[kEncoderCount] = {
        {0, 1},  // LiftLeft
        {1, 1},  // LiftRight
        {0, 2},  // TiltLeft
        {1, 2},  // TiltRight
        {0, 3},  // JawsLeft
        {1, 3},  // JawsRight
    };

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

    for (;;)
    {
        modbus::Request request;
        while (xQueueReceive(bus.commands, &request, 0) == pdTRUE)
            bus.mbus.submit(request);

        const uint32_t now = clock_.now_ms();
        if (now - last_heartbeat_ms >= kHeartbeatPeriodMs && bus.mbus.submit(enc::heartbeat_request()))
            last_heartbeat_ms = now;

        if (bus.mbus.step())
            refresh_stats(bus, bus_index);
        else
            vTaskDelay(1);
    }
}

// Runs before any device is registered, so each step() executes exactly the
// probe just submitted.
void EncoderBus::discover(Bus& bus, size_t bus_index)
{
    constexpr size_t kSlots = kDiscoveryLastSlave - kDiscoveryFirstSlave + 1;
    uint8_t hits[kSlots] = {};
    const unsigned bus_no = static_cast<unsigned>(bus_index + 1);

    ESP_LOGI(TAG, "bus %u: discovering slaves %u-%u, %u rounds", bus_no, kDiscoveryFirstSlave, kDiscoveryLastSlave,
             kDiscoveryRounds);

    for (uint8_t round = 0; round < kDiscoveryRounds; ++round)
    {
        for (uint8_t slave = kDiscoveryFirstSlave; slave <= kDiscoveryLastSlave; ++slave)
        {
            TickType_t wake = xTaskGetTickCount();
            const bool answered = probe(bus, slave, true);
            if (answered)
                ++hits[slave - kDiscoveryFirstSlave];
            vTaskDelayUntil(&wake, pdMS_TO_TICKS(kDiscoveryProbeMs));
            if (answered)
                probe(bus, slave, false);
        }
    }

    size_t registered = 0;
    for (uint8_t slave = kDiscoveryFirstSlave; slave <= kDiscoveryLastSlave; ++slave)
    {
        const uint8_t count = hits[slave - kDiscoveryFirstSlave];

        size_t index = kEncoderCount;
        for (size_t i = 0; i < kEncoderCount; ++i)
            if (kPlacement[i].bus == bus_index && kPlacement[i].slave == slave)
                index = i;

        if (index == kEncoderCount)
        {
            if (count > 0)
                ESP_LOGW(TAG, "bus %u: slave %u answered %u/%u but has no placement", bus_no, slave, count,
                         kDiscoveryRounds);
            continue;
        }

        if (count == 0)
        {
            ESP_LOGW(TAG, "bus %u: %s (slave %u) missing", bus_no, kNames[index], slave);
            continue;
        }
        if (count < kDiscoveryRounds)
            ESP_LOGW(TAG, "bus %u: %s (slave %u) flaky, answered %u/%u", bus_no, kNames[index], slave, count,
                     kDiscoveryRounds);
        else
            ESP_LOGI(TAG, "bus %u: %s (slave %u) found", bus_no, kNames[index], slave);

        {
            Lock lock(mutex_);
            readings_[index].present = true;
        }

        if (!(enabled_ & (1u << index)))
            continue;

        enc::DeviceConfig config;
        config.slave = slave;
        config.angle_period_ms = kAnglePeriodMs;
        config.status_period_ms = kStatusPeriodMs;
        config.on_angle = &EncoderBus::on_angle;
        config.on_status = &EncoderBus::on_status;
        config.on_state_change = &EncoderBus::on_state_change;
        config.ctx = &slots_[index];

        if (bus.mbus.add_device(enc::make_device(config)))
            ++registered;
        else
            ESP_LOGE(TAG, "failed to register %s", kNames[index]);
    }

    if (registered < kDevicesPerBus)
        ESP_LOGW(TAG, "bus %u: degraded, polling %u/%u encoders", bus_no, static_cast<unsigned>(registered),
                 static_cast<unsigned>(kDevicesPerBus));
}

bool EncoderBus::probe(Bus& bus, uint8_t slave, bool on)
{
    modbus::Result result = modbus::Result::Timeout;
    if (!bus.mbus.submit(enc::discovery_request(slave, on, &EncoderBus::on_probe_done, &result)))
        return false;
    bus.mbus.step();
    return result == modbus::Result::Ok || result == modbus::Result::ExceptionReply;
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
        reading.degrees = angle.raw_counts * (360.0f / 4096.0f);
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
