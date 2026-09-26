#pragma once

#include <cstdint>
#include <hi_can_address.hpp>
#include <optional>
#include <string>
#include <vector>

namespace can_diag
{
    struct cli_options_t
    {
        std::string iface = "vcan0";

        std::optional<std::string> preset_name{};
        std::optional<hi_can::addressing::raw_address_t> explicit_id{};
        std::optional<uint8_t> system{};
        std::optional<uint8_t> subsystem{};
        std::optional<uint8_t> device{};
        std::optional<uint8_t> group{};
        std::optional<uint8_t> parameter{};

        std::optional<bool> rtr_override{};
        std::optional<std::vector<uint8_t>> data{};

        bool listen = false;
        unsigned timeout_ms = 500;
        bool raw = false;
        bool list_presets = false;
        bool help = false;
    };

    /// @brief Parses argv into cli_options_t
    /// @exception std::invalid_argument on any malformed/conflicting arguments
    cli_options_t parse_args(int argc, const char** argv);

    void print_usage();
}  // namespace can_diag
