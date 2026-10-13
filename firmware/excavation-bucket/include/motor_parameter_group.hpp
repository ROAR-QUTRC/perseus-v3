/**
 * @file motor_parameter_group.hpp
 * @details Provides class for managing the hi_can transmission and callbacks for encoder readings (GET_ANGLE) of a single bucket actuator. Commands are per bank, via MotorBankParameterGroup.
 */
#pragma once

#include "hi_can_address.hpp"
#include "hi_can_parameter.hpp"
#include "motor_driver.hpp"

class MotorParameterGroup : public hi_can::parameters::ParameterGroup
{
public:
    MotorParameterGroup(hi_can::addressing::excavation::bucket::controller::encoder_group encoder_group, MotorDriver& motor_driver);

private:
    hi_can::addressing::excavation::bucket::controller::encoder_group _encoder_group;
    MotorDriver& _motor_driver;
};