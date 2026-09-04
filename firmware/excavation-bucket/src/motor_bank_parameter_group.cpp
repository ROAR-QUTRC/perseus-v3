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

MotorBankParameterGroup::MotorBankParameterGroup(const hi_can::addressing::excavation::bucket::controller::group bank_group,
                                                 MotorBank& motor_bank)
    : _bank_group(bank_group),
      _motor_bank(motor_bank)
{
    _transmissions = {
        {static_cast<flagged_address_t>(standard_address_t{
             DEVICE_ADDRESS, static_cast<uint8_t>(group::LIFT),
             static_cast<uint8_t>(bank_parameter::CURRENT)}),
         PacketManager::transmission_config_t{
             .generator = ([this]()
                           { return this->_motor_bank.get_current(); }),
             .interval = 500ms}},
    };
    _callbacks = {
        std::make_pair(
            filter_t{
                static_cast<flagged_address_t>(standard_address_t{
                    DEVICE_ADDRESS, static_cast<uint8_t>(_bank_group), static_cast<uint8_t>(bank_parameter::SPEED)}),
            },
            PacketManager::callback_config_t{
                .data_callback = ([this](const Packet& packet)
                                  {
                 parameters::excavation::bucket::controller::speed_t speed{packet.get_data()};
                 this->_motor_bank.set_speed(speed.value); }),
                .timeout_callback = [this]()
                { this->_motor_bank.set_speed(0); },
                .timeout = 200ms,
            }),
    };
}
