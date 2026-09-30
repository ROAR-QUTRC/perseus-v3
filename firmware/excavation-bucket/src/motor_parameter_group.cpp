#include "motor_parameter_group.hpp"

#include <chrono>
#include <tuple>

#include "hi_can.hpp"

using namespace std::chrono_literals;
using namespace hi_can;
using namespace hi_can::addressing;
using namespace hi_can::addressing::excavation;
using namespace hi_can::addressing::excavation::bucket;
using namespace hi_can::addressing::excavation::bucket::controller;

constexpr standard_address_t DEVICE_ADDRESS{
    excavation::SYSTEM_ID,
    excavation::bucket::SUBSYSTEM_ID,
    excavation::bucket::controller::DEVICE_ID,
};

MotorParameterGroup::MotorParameterGroup(hi_can::addressing::excavation::bucket::controller::encoder_group encoder_group, MotorDriver& motor_driver)
    : _encoder_group(encoder_group),
      _motor_driver(motor_driver)
{
    _transmissions = {
        std::make_pair(
            static_cast<flagged_address_t>(standard_address_t{
                DEVICE_ADDRESS, static_cast<uint8_t>(_encoder_group),
                static_cast<uint8_t>(controller::encoder_parameter::GET_ANGLE)}),
            // TODO: this keeps sending the last cached angle after the encoder stops
            // answering, so ROS never sees it go stale. Only send while the reading is fresh.
            PacketManager::transmission_config_t{
                .generator = ([this]()
                              {
                parameters::excavation::bucket::controller::position_t position{this->_motor_driver.get_current_position()};
                return position.serialize_data(); }),
                .interval = 50ms}),
    };
}