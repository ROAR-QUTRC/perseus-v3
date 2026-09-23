#pragma once

#include <functional>
#include <hi_can_address.hpp>
#include <hi_can_parameter.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// Known hi-can packets, used both as named --preset addresses for building
// requests and as a decode/label lookup for responses.
//
// To add a new decodable packet type:
//   1. Locate (or add) the standard_address_t/enum combination for it in
//      hi_can_address.hpp's addressing namespace.
//   2. Locate (or add) the corresponding payload typedef in
//      hi_can_parameter.hpp (SimpleSerializable<T>/scaled_int16_t<N>/etc,
//      all constructible from a std::vector<uint8_t>).
//   3. Push a new known_packet_t entry below with a decode lambda that
//      constructs that type from the data vector and formats its value(s).
// Anything not registered here still works, just falls back to a raw hex
// dump instead of a decoded value.
namespace can_diag
{
    struct known_packet_t
    {
        std::string name;
        hi_can::addressing::raw_address_t address;
        bool request_is_rtr;
        std::string description;
        std::function<std::string(const std::vector<uint8_t>&)> decode;
    };

    namespace detail
    {
        // Decodes a data buffer using one of hi_can's existing
        // BidirectionalSerializable payload structs (they all throw
        // std::invalid_argument on a size mismatch).
        template <typename T>
        std::string decode_wrapped(const std::vector<uint8_t>& data)
        {
            try
            {
                T value{data};
                return std::to_string(value.value);
            }
            catch (const std::exception& e)
            {
                return std::string("<decode error: ") + e.what() + ">";
            }
        }
    }  // namespace detail

    inline const std::vector<known_packet_t>& known_packets()
    {
        static const std::vector<known_packet_t> table = []
        {
            namespace bucket_addr = hi_can::addressing::excavation::bucket;
            namespace bucket_param = hi_can::parameters::excavation::bucket;

            constexpr uint8_t SYSTEM_ID = hi_can::addressing::excavation::SYSTEM_ID;
            constexpr uint8_t SUBSYSTEM_ID = bucket_addr::SUBSYSTEM_ID;
            constexpr uint8_t DEVICE_ID = bucket_addr::controller::DEVICE_ID;

            auto addr = [&](uint8_t group, uint8_t parameter) -> hi_can::addressing::raw_address_t
            {
                return static_cast<hi_can::addressing::raw_address_t>(
                    hi_can::addressing::standard_address_t(SYSTEM_ID, SUBSYSTEM_ID, DEVICE_ID,
                                                           group, parameter));
            };

            std::vector<known_packet_t> t;

            struct encoder_entry
            {
                bucket_addr::controller::encoder_group group;
                const char* name;
            };
            static constexpr encoder_entry encoders[] = {
                {bucket_addr::controller::encoder_group::LIFT_L, "lift_l"},
                {bucket_addr::controller::encoder_group::LIFT_R, "lift_r"},
                {bucket_addr::controller::encoder_group::TILT_L, "tilt_l"},
                {bucket_addr::controller::encoder_group::TILT_R, "tilt_r"},
                {bucket_addr::controller::encoder_group::JAWS_L, "jaws_l"},
                {bucket_addr::controller::encoder_group::JAWS_R, "jaws_r"},
            };
            for (const auto& e : encoders)
            {
                t.push_back({
                    std::string("excavation.bucket.encoder.") + e.name + ".get_angle",
                    addr(static_cast<uint8_t>(e.group),
                         static_cast<uint8_t>(bucket_addr::controller::encoder_parameter::GET_ANGLE)),
                    /*request_is_rtr=*/true,
                    std::string(e.name) + " encoder angle (RTR request)",
                    detail::decode_wrapped<bucket_param::controller::position_t>,
                });
            }

            struct bank_entry
            {
                bucket_addr::controller::bank_group group;
                const char* name;
            };
            static constexpr bank_entry banks[] = {
                {bucket_addr::controller::bank_group::LIFT, "lift"},
                {bucket_addr::controller::bank_group::TILT, "tilt"},
                {bucket_addr::controller::bank_group::JAWS, "jaws"},
            };
            for (const auto& b : banks)
            {
                t.push_back({
                    std::string("excavation.bucket.motor_bank.") + b.name + ".get_current",
                    addr(static_cast<uint8_t>(b.group),
                         static_cast<uint8_t>(bucket_addr::controller::bank_parameter::GET_CURRENT)),
                    /*request_is_rtr=*/false,
                    std::string(b.name) +
                        " motor bank current (periodic broadcast only - use --listen, not --rtr)",
                    detail::decode_wrapped<bucket_param::controller::current_t>,
                });
                t.push_back({
                    std::string("excavation.bucket.motor_bank.") + b.name + ".set_speed",
                    addr(static_cast<uint8_t>(b.group),
                         static_cast<uint8_t>(bucket_addr::controller::bank_parameter::SET_SPEED)),
                    /*request_is_rtr=*/false,
                    std::string(b.name) + " motor bank speed (send with --data)",
                    detail::decode_wrapped<bucket_param::controller::speed_t>,
                });
            }

            return t;
        }();
        return table;
    }

    inline std::optional<known_packet_t> find_preset_by_name(std::string_view name)
    {
        for (const auto& entry : known_packets())
            if (entry.name == name)
                return entry;
        return std::nullopt;
    }

    inline std::optional<known_packet_t>
    find_preset_by_address(hi_can::addressing::raw_address_t address)
    {
        for (const auto& entry : known_packets())
            if (entry.address == address)
                return entry;
        return std::nullopt;
    }
}  // namespace can_diag
