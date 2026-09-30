#include "motor_bank_parameter_group.hpp"

#include <chrono>
#include <tuple>

#include "encoder_bus.hpp"
#include "esp_log.h"
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

static const char* const TAG = "bank";

MotorBankParameterGroup::MotorBankParameterGroup(const hi_can::addressing::excavation::bucket::controller::bank_group bank_group,
                                                 MotorBank& motor_bank)
    : _bank_group(bank_group),
      _motor_bank(motor_bank)
{
    // TODO: GET_FAULT is never sent (MotorBank::get_fault() is unused), so ROS keeps
    // BankState.fault false while the bank looks fresh. Plan: send it as a 1-byte
    // fault bitfield, 0 = healthy (driver fault, encoder lost / no magnet / zero not
    // saved per side), with the bits defined in hi-can so ROS and the TUI can name them.
    _transmissions = {
        std::make_pair(
            static_cast<flagged_address_t>(standard_address_t{
                DEVICE_ADDRESS, static_cast<uint8_t>(_bank_group),
                static_cast<uint8_t>(bank_parameter::GET_CURRENT)}),
            PacketManager::transmission_config_t{
                .generator = ([this]()
                              { return this->_motor_bank.get_current(); }),
                .interval = 50ms}),  // same rate as GET_ANGLE / GET_POSITION
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
        // Zeroes both of the bank's encoders; each saves its new offset to flash.
        std::make_pair(
            filter_t{
                static_cast<flagged_address_t>(standard_address_t{
                    DEVICE_ADDRESS, static_cast<uint8_t>(_bank_group),
                    static_cast<uint8_t>(bank_parameter::SET_ZERO_POS)}),
            },
            PacketManager::callback_config_t{
                .data_callback = ([this](const Packet& packet)
                                  {
                                    // The filter ignores the RTR flag; a request for data must not zero.
                                    if (packet.get_is_rtr())
                                        return;
                                    // A position target means something else once the angle is re-zeroed.
                                    this->_motor_bank.stop();
                                    const bool a = encoder_bus().zero(this->_motor_bank.get_driver_A().encoder_id());
                                    const bool b = encoder_bus().zero(this->_motor_bank.get_driver_B().encoder_id());
                                    ESP_LOGI(TAG, "SET_ZERO_POS bank %u: %s", static_cast<unsigned>(this->_bank_group),
                                             (a && b) ? "zero queued for both encoders" : "not queued (encoder bus not running or busy)"); }),
            }),
    };
}
