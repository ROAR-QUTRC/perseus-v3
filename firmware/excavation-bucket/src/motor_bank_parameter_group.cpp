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
        {static_cast<flagged_address_t>(standard_address_t{
             DEVICE_ADDRESS, static_cast<uint8_t>(_bank_group),
             static_cast<uint8_t>(bank_parameter::GET_CURRENT)}),
         PacketManager::transmission_config_t{
             .generator = ([this]()
                           { return this->_motor_bank.get_current(); }),
             .interval = 500ms}},
        {static_cast<flagged_address_t>(standard_address_t{
             DEVICE_ADDRESS, static_cast<uint8_t>(_bank_group),
             static_cast<uint8_t>(bank_parameter::GET_POSITION)}),
         PacketManager::transmission_config_t{
             .generator = ([this]()
                           { return this->_motor_bank.get_position(); }),
             .interval = 500ms}},
        {static_cast<flagged_address_t>(standard_address_t{
             DEVICE_ADDRESS, static_cast<uint8_t>(_bank_group),
             static_cast<uint8_t>(bank_parameter::GET_FAULT)}),
         PacketManager::transmission_config_t{
             .generator = ([this]()
                           { return this->_motor_bank.get_fault(); }),
             .interval = 500ms}},
    };
    _callbacks = {
        std::make_pair(
            filter_t{
                static_cast<flagged_address_t>(standard_address_t{
                    DEVICE_ADDRESS, static_cast<uint8_t>(_bank_group), static_cast<uint8_t>(bank_parameter::SET_SPEED)}),
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
        std::make_pair(
            filter_t{
                static_cast<flagged_address_t>(standard_address_t{
                    DEVICE_ADDRESS, static_cast<uint8_t>(_bank_group), static_cast<uint8_t>(bank_parameter::SET_POSITION)}),
            },
            PacketManager::callback_config_t{
                .data_callback = ([this](const Packet& packet)
                                  {
                 parameters::excavation::bucket::controller::position_t position{packet.get_data()};
                 this->_motor_bank.set_position_setpoint(position.value); }),
                .timeout_callback = [this]()
                { this->_motor_bank.set_speed(0); },
                .timeout = 200ms,
            }),
        std::make_pair(
            filter_t{
                static_cast<flagged_address_t>(standard_address_t{
                    DEVICE_ADDRESS, static_cast<uint8_t>(_bank_group), static_cast<uint8_t>(bank_parameter::SET_PID_PARAMS)}),
            },
            PacketManager::callback_config_t{
                // No .timeout_callback/.timeout - gains are config, not a
                // keep-alive setpoint; a stale value just means the loop keeps
                // using the last-configured gains.
                .data_callback = ([this](const Packet& packet)
                                  {
                 parameters::excavation::bucket::controller::pid_params_t params{packet.get_data()};
                 this->_motor_bank.set_pid_gains(PidController::Gains{
                     static_cast<double>(params.K_p) / parameters::excavation::bucket::controller::PID_GAIN_SCALE,
                     static_cast<double>(params.K_i) / parameters::excavation::bucket::controller::PID_GAIN_SCALE,
                     static_cast<double>(params.K_d) / parameters::excavation::bucket::controller::PID_GAIN_SCALE}); }),
            }),
    };
}
