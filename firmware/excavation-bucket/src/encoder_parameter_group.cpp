#include "encoder_parameter_group.hpp"

#include "hi_can.hpp"
#include "hi_can_address.hpp"

using namespace hi_can;
using namespace hi_can::addressing;
using namespace hi_can::addressing::excavation::bucket::controller;

constexpr standard_address_t DEVICE_ADDRESS{
    excavation::SYSTEM_ID,
    excavation::bucket::SUBSYSTEM_ID,
    excavation::bucket::controller::DEVICE_ID,
};

EncoderParameterGroup::EncoderParameterGroup(
    const EncoderId encoder_id,
    const hi_can::addressing::excavation::bucket::controller::encoder_group encoder_group,
    hi_can::FilteredCanInterface& can_interface)
    : _encoder_id(encoder_id),
      _encoder_group(encoder_group),
      _can_interface(can_interface)
{
    _callbacks = {
        std::make_pair(
            filter_t{
                static_cast<flagged_address_t>(standard_address_t{
                    DEVICE_ADDRESS, static_cast<uint8_t>(_encoder_group),
                    static_cast<uint8_t>(encoder_parameter::GET_ANGLE)}),
            },
            PacketManager::callback_config_t{
                .data_callback = ([this](const Packet& packet)
                                  {
                    // Only ever reply to a request (RTR); a stray data frame
                    // landing on this address gets no response.
                    if (!packet.get_is_rtr())
                        return;

                    EncoderReading reading;
                    if (!encoder_bus().get(this->_encoder_id, &reading) || !reading.angle_valid)
                        return;  // not ready yet - stay silent rather than guess

                    parameters::excavation::bucket::controller::position_t position{
                        static_cast<int16_t>(reading.raw_counts)};

                    const standard_address_t address{
                        DEVICE_ADDRESS, static_cast<uint8_t>(this->_encoder_group),
                        static_cast<uint8_t>(encoder_parameter::GET_ANGLE)};
                    this->_can_interface.transmit(Packet(
                        static_cast<flagged_address_t>(address), position.serialize_data())); }),
            }),
    };
}
