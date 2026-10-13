#pragma once

#include "encoder_bus.hpp"
#include "hi_can_parameter.hpp"

// Replies to a GET_ANGLE request (CAN RTR frame) for one encoder with its
// latest cached reading from encoder_bus(). Never sends anything on its own -
// only in response to a request.
class EncoderParameterGroup : public hi_can::parameters::ParameterGroup
{
public:
    EncoderParameterGroup(EncoderId encoder_id,
                          hi_can::addressing::excavation::bucket::controller::encoder_group encoder_group,
                          hi_can::FilteredCanInterface& can_interface);

private:
    const EncoderId _encoder_id;
    const hi_can::addressing::excavation::bucket::controller::encoder_group _encoder_group;
    hi_can::FilteredCanInterface& _can_interface;
};
