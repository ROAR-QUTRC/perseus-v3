#include "motor_bank_parameter_group.hpp"

#include <chrono>
#include <tuple>

#include "hi_can.hpp"
#include "hi_can_address.hpp"
using namespace std::chrono_literals;
using namespace hi_can;
using namespace hi_can::addressing;
using namespace hi_can::addressing::excavation::bucket::controller;

constexpr standard_address_t DEVICE_ADDRESS{
    excavation::SYSTEM_ID,
    excavation::bucket::SUBSYSTEM_ID,
    excavation::bucket::controller::DEVICE_ID,
};

MotorBankParameterGroup::MotorBankParameterGroup(const hi_can::addressing::excavation::bucket::controller::bank_group bank_group,
                                                 MotorBank& motor_bank)
    : _bank_group(bank_group),
      _motor_bank(motor_bank)
{
    _transmissions = {
        std::make_pair(
            static_cast<flagged_address_t>(standard_address_t{
                DEVICE_ADDRESS, static_cast<uint8_t>(_bank_group),
                static_cast<uint8_t>(bank_parameter::GET_CURRENT)}),
            PacketManager::transmission_config_t{
                .generator = ([this]()
                              { return this->_motor_bank.get_current(); }),
                .interval = 500ms}),
        std::make_pair(
            static_cast<flagged_address_t>(standard_address_t{
                DEVICE_ADDRESS, static_cast<uint8_t>(_bank_group),
                static_cast<uint8_t>(bank_parameter::GET_POSITION)}),
            PacketManager::transmission_config_t{.generator = ([this]()
                                                               {
                                                                    parameters::excavation::bucket::controller::position_t position{this->_motor_bank.get_current_position()};
                                                                    return position.serialize_data(); }),
                                                 .interval = 50ms}),
    };
    _callbacks = {
        std::make_pair(
            filter_t{
                static_cast<flagged_address_t>(standard_address_t{
                    DEVICE_ADDRESS, static_cast<uint8_t>(_bank_group),
                    static_cast<uint8_t>(bank_parameter::SET_SPEED)}),
            },
            PacketManager::callback_config_t{
                .data_callback = ([this](const Packet& packet)
                                  {
                                    const auto& raw_data = packet.get_data();
                                    if (raw_data.size() != sizeof(int16_t))
                                        return;
                                    parameters::excavation::bucket::controller::speed_t speed;
                                    speed.deserialize_data(raw_data);
                                    this->_motor_bank.set_speed(speed.value); }),
                .timeout_callback = [this]()
                { this->_motor_bank.set_speed(0); },
                .timeout = 200ms,
            }),
        std::make_pair(
            filter_t{
                static_cast<flagged_address_t>(standard_address_t{
                    DEVICE_ADDRESS, static_cast<uint8_t>(_bank_group),
                    static_cast<uint8_t>(bank_parameter::SET_POSITION)}),
            },
            PacketManager::callback_config_t{
                .data_callback = ([this](const Packet& packet)
                                  {
                                    const auto& raw_data = packet.get_data();
                                    if (raw_data.size() != sizeof(int16_t))
                                        return;
                                    parameters::excavation::bucket::controller::position_t position;
                                    position.deserialize_data(raw_data);
                                    this->_motor_bank.set_target_position(position.value); }),
                .timeout = 200ms,
            }),
    };
}
