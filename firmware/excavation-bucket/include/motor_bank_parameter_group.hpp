#pragma once

#include "excavation_config.hpp"
#include "hi_can_address.hpp"
#include "hi_can_parameter.hpp"
#include "motor_bank.hpp"

inline constexpr hi_can::addressing::standard_address_t kBucketAddress{
    hi_can::addressing::excavation::SYSTEM_ID,
    hi_can::addressing::excavation::bucket::SUBSYSTEM_ID,
    bucket_can::DEVICE_ID,
};

/**
 * @brief All of one bank's CAN traffic.
 * @details Sends GET_CURRENT, GET_POSITION and each encoder's GET_ANGLE every 50 ms; handles SET_SPEED,
 * SET_POSITION and SET_ZERO_POS. Its callbacks only hold on to the bank, so the group itself can be a temporary
 * handed to PacketManager::add_group().
 */
class MotorBankParameterGroup : public hi_can::parameters::ParameterGroup
{
public:
    MotorBankParameterGroup(const BankConfig& config, MotorBank& bank);
};
