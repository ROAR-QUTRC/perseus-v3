#include "motor_bank_parameter_group.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>

#include "hi_can.hpp"

using namespace std::chrono_literals;
using namespace hi_can;
using namespace hi_can::addressing;
using namespace bucket_can;
namespace params = hi_can::parameters::excavation::bucket::controller;

namespace
{
    constexpr auto kReportInterval = 50ms;
    // hi-can stops the bank after 3 of these with no SET_SPEED.
    constexpr auto kSpeedTimeout = 200ms;

    template <typename Group, typename Parameter>
    flagged_address_t address_of(Group group, Parameter parameter)
    {
        return static_cast<flagged_address_t>(
            standard_address_t{kBucketAddress, static_cast<uint8_t>(group), static_cast<uint8_t>(parameter)});
    }

    // SET_SPEED and SET_POSITION each carry one int16; anything else is ignored.
    std::optional<int16_t> read_int16(const Packet& packet)
    {
        if (packet.get_data().size() != sizeof(int16_t))
            return std::nullopt;
        params::position_t value;
        value.deserialize_data(packet.get_data());
        return value.value;
    }

    std::vector<uint8_t> position_data(int16_t position) { return params::position_t{position}.serialize_data(); }
}  // namespace

MotorBankParameterGroup::MotorBankParameterGroup(const BankConfig& config, MotorBank& bank)
{
    // TODO: GET_FAULT is never sent (MotorBank::is_in_fault() is only used by homing), so ROS keeps
    // BankState.fault false while the bank looks fresh. Plan: send it as a 1-byte
    // fault bitfield, 0 = healthy (driver fault, encoder lost / no magnet / zero not
    // saved per side), with the bits defined in hi-can so ROS and the TUI can name them.
    const auto report = [this](flagged_address_t address, PacketManager::data_generator_t generator)
    {
        _transmissions.emplace_back(address, PacketManager::transmission_config_t{.generator = std::move(generator), .interval = kReportInterval});
    };

    // GET_CURRENT is in mA, what ROS's bucket_hardware decodes.
    const auto current = [&bank]
    {
        const float milliamps = std::clamp(bank.current_amps() * 1000.0f, 0.0f, static_cast<float>(std::numeric_limits<uint16_t>::max()));
        return params::current_t{static_cast<uint16_t>(std::lround(milliamps))}.serialize_data();
    };
    const auto position = [&bank]
    {
        return position_data(bank.get_current_position());
    };
    report(address_of(config.can_group, bank_parameter::GET_CURRENT), current);
    report(address_of(config.can_group, bank_parameter::GET_POSITION), position);

    // TODO: this keeps sending the last cached angle after the encoder stops
    // answering, so ROS never sees it go stale. Only send while the reading is fresh.
    for (const DriverConfig& driver : {config.a, config.b})
    {
        const auto angle = [encoder = driver.encoder]
        {
            return position_data(MotorBank::encoder_position(encoder));
        };
        report(address_of(driver.can_group, encoder_parameter::GET_ANGLE), angle);
    }

    const auto on = [this, &config](bank_parameter parameter, PacketManager::callback_config_t callbacks)
    {
        _callbacks.emplace_back(filter_t{address_of(config.can_group, parameter)}, std::move(callbacks));
    };

    const auto set_speed = [&bank](const Packet& packet)
    {
        if (const auto speed = read_int16(packet))
            bank.set_speed(*speed);
    };
    const auto speed_timeout = [&bank]
    {
        bank.set_speed(0);
    };
    const auto set_position = [&bank](const Packet& packet)
    {
        if (const auto target = read_int16(packet))
            bank.set_target_position(*target);
    };
    const auto zero = [&bank](const Packet& packet)
    {
        // The filter ignores the RTR flag; a request for data must not zero.
        if (!packet.get_is_rtr())
            bank.zero();
    };
    on(bank_parameter::SET_SPEED, {.data_callback = set_speed, .timeout_callback = speed_timeout, .timeout = kSpeedTimeout});
    on(bank_parameter::SET_POSITION, {.data_callback = set_position});
    on(bank_parameter::SET_ZERO_POS, {.data_callback = zero});
}
