#include <can_diag/cli_args.hpp>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace can_diag
{
    namespace
    {
        // Parses a decimal or "0x"-prefixed hex integer.
        unsigned long parse_unsigned(const std::string& text, const std::string& what)
        {
            try
            {
                size_t consumed = 0;
                unsigned long value = std::stoul(text, &consumed, 0);
                if (consumed != text.size())
                    throw std::invalid_argument("trailing garbage");
                return value;
            }
            catch (const std::exception&)
            {
                throw std::invalid_argument("Invalid value for " + what + ": '" + text + "'");
            }
        }

        uint8_t parse_field(const std::string& text, unsigned bits, const std::string& flag)
        {
            const unsigned long value = parse_unsigned(text, flag);
            const unsigned long max_value = (1UL << bits) - 1UL;
            if (value > max_value)
                throw std::invalid_argument(flag + " value " + text + " exceeds " +
                                            std::to_string(bits) + "-bit field (max " +
                                            std::to_string(max_value) + ")");
            return static_cast<uint8_t>(value);
        }

        std::vector<uint8_t> parse_data(const std::string& text)
        {
            std::vector<uint8_t> bytes;
            std::stringstream stream(text);
            std::string token;
            while (std::getline(stream, token, ','))
            {
                if (token.empty())
                    throw std::invalid_argument("--data contains an empty byte entry");
                const unsigned long value = parse_unsigned(token, "--data byte '" + token + "'");
                if (value > 0xFFUL)
                    throw std::invalid_argument("--data byte '" + token + "' does not fit in a byte");
                bytes.push_back(static_cast<uint8_t>(value));
            }
            if (bytes.size() > hi_can::addressing::MAX_PACKET_LEN)
                throw std::invalid_argument("--data has " + std::to_string(bytes.size()) +
                                            " bytes, max is " +
                                            std::to_string(hi_can::addressing::MAX_PACKET_LEN));
            return bytes;
        }
    }  // namespace

    cli_options_t parse_args(int argc, const char** argv)
    {
        cli_options_t opts;
        std::vector<std::string> args(argv + 1, argv + argc);

        bool rtr_flag = false;
        bool no_rtr_flag = false;

        auto next_value = [&](size_t& i, const std::string& flag) -> std::string
        {
            if (i + 1 >= args.size())
                throw std::invalid_argument("Missing value for " + flag);
            return args[++i];
        };

        for (size_t i = 0; i < args.size(); ++i)
        {
            const std::string& arg = args[i];

            if (arg == "-h" || arg == "--help")
                opts.help = true;
            else if (arg == "-i" || arg == "--iface")
                opts.iface = next_value(i, arg);
            else if (arg == "--preset")
                opts.preset_name = next_value(i, arg);
            else if (arg == "--id")
                opts.explicit_id = static_cast<hi_can::addressing::raw_address_t>(
                    parse_unsigned(next_value(i, arg), arg));
            else if (arg == "--system")
                opts.system = parse_field(next_value(i, arg), hi_can::addressing::SYSTEM_ADDRESS_BITS, arg);
            else if (arg == "--subsystem")
                opts.subsystem =
                    parse_field(next_value(i, arg), hi_can::addressing::SUBSYSTEM_ADDRESS_BITS, arg);
            else if (arg == "--device")
                opts.device = parse_field(next_value(i, arg), hi_can::addressing::DEVICE_ADDRESS_BITS, arg);
            else if (arg == "--group")
                opts.group = parse_field(next_value(i, arg), hi_can::addressing::GROUP_ADDRESS_BITS, arg);
            else if (arg == "--parameter")
                opts.parameter =
                    parse_field(next_value(i, arg), hi_can::addressing::PARAM_ADDRESS_BITS, arg);
            else if (arg == "--rtr")
                rtr_flag = true;
            else if (arg == "--no-rtr")
                no_rtr_flag = true;
            else if (arg == "--data")
                opts.data = parse_data(next_value(i, arg));
            else if (arg == "--listen")
                opts.listen = true;
            else if (arg == "--timeout")
                opts.timeout_ms = static_cast<unsigned>(parse_unsigned(next_value(i, arg), arg));
            else if (arg == "--raw")
                opts.raw = true;
            else if (arg == "--list-presets")
                opts.list_presets = true;
            else
                throw std::invalid_argument("Unknown argument: " + arg);
        }

        if (opts.help || opts.list_presets)
            return opts;

        if (rtr_flag && no_rtr_flag)
            throw std::invalid_argument("--rtr and --no-rtr are mutually exclusive");
        if (rtr_flag)
            opts.rtr_override = true;
        if (no_rtr_flag)
            opts.rtr_override = false;

        if (opts.preset_name && opts.explicit_id)
            throw std::invalid_argument("--preset and --id are mutually exclusive");

        if (opts.data && rtr_flag)
            throw std::invalid_argument(
                "--data cannot be combined with --rtr (RTR frames carry no payload)");

        if (opts.listen && (opts.data || rtr_flag))
            throw std::invalid_argument("--listen does not transmit, so --data/--rtr have no effect");

        if (!opts.preset_name && !opts.explicit_id && !opts.system && !opts.subsystem &&
            !opts.device && !opts.group && !opts.parameter)
            throw std::invalid_argument(
                "No target address given: specify --preset, --id, or address fields "
                "(--system/--subsystem/--device/--group/--parameter)");

        return opts;
    }

    void print_usage()
    {
        std::cout << "can_diag - generic hi-can diagnostic tool\n"
                     "\n"
                     "Sends an arbitrary hi-can packet (RTR request or data frame) and prints/decodes\n"
                     "whatever response arrives.\n"
                     "\n"
                     "Usage: can_diag [options]\n"
                     "\n"
                     "Addressing (choose one, or combine with field overrides):\n"
                     "  --preset <name>        Use a named entry from --list-presets\n"
                     "  --id <hex>              Raw 29-bit address\n"
                     "  --system/--subsystem/--device/--group/--parameter <hex|dec>\n"
                     "                          Individual address fields (override the base address's\n"
                     "                          corresponding field when combined with --preset/--id)\n"
                     "\n"
                     "Options:\n"
                     "  -i, --iface <name>      CAN interface (default: vcan0)\n"
                     "  --rtr / --no-rtr        Force the RTR flag on/off (default: preset's default, else off)\n"
                     "  --data <hex,hex,...>    Outgoing payload bytes, e.g. --data 01,02 (max 8, not with --rtr)\n"
                     "  --listen                Don't transmit; just wait for/print a matching packet\n"
                     "  --timeout <ms>          Max time to wait for a response (default 500; 0 = don't wait)\n"
                     "  --raw                   Force raw hex output even if a decoder is registered\n"
                     "  --list-presets          Print known preset addresses and exit\n"
                     "  -h, --help              Print this message and exit\n";
    }
}  // namespace can_diag
