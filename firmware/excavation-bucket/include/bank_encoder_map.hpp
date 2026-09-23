#pragma once
// Maps each motor bank to the pair of physical encoders (left, right) that
// feed its closed-loop position control and GET_POSITION report. Mirrors
// encoder_group_for(Axis, Side) in the ROS host's can_board_interface.cpp -
// keep both in sync if a joint's wiring changes.

#include "encoder_bus.hpp"
#include "hi_can_address.hpp"

struct BankEncoderPair
{
    EncoderId left;
    EncoderId right;
};

constexpr BankEncoderPair encoder_pair_for(
    hi_can::addressing::excavation::bucket::controller::bank_group bank)
{
    using bank_group = hi_can::addressing::excavation::bucket::controller::bank_group;
    switch (bank)
    {
    case bank_group::LIFT:
        return {EncoderId::LiftLeft, EncoderId::LiftRight};
    case bank_group::TILT:
        return {EncoderId::TiltLeft, EncoderId::TiltRight};
    case bank_group::JAWS:
        return {EncoderId::JawsLeft, EncoderId::JawsRight};
    }
    return {EncoderId::LiftLeft, EncoderId::LiftRight};  // unreachable, silences -Wreturn-type
}
