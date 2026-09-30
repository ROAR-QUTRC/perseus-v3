#include <can_diag/cli_args.hpp>
#include <can_diag/known_packets.hpp>
#include <chrono>
#include <format>
#include <hi_can_raw.hpp>
#include <iostream>
#include <thread>

using namespace hi_can;
using std::cout, std::endl;

namespace
{
    void print_address(const addressing::flagged_address_t& address, bool is_outgoing)
    {
        const addressing::standard_address_t fields(address.address);
        cout << std::format(
                    "{} id={:#010x} (system={:#04x} subsystem={:#04x} device={:#04x} "
                    "group={:#04x} parameter={:#04x}) rtr={} extended={}",
                    is_outgoing ? "TX" : "RX", address.address, fields.system, fields.subsystem,
                    fields.device, fields.group, fields.parameter, address.is_rtr,
                    address.is_extended)
             << endl;
    }

    void print_data(const std::vector<uint8_t>& data)
    {
        if (data.empty())
            return;
        cout << "  data:";
        for (const auto byte : data)
            cout << std::format(" {:#04x}", byte);
        cout << endl;
    }

    void print_presets()
    {
        cout << std::format("{:<55} {:>12} {:>5}  {}", "NAME", "ADDRESS", "RTR", "DESCRIPTION")
             << endl;
        for (const auto& entry : can_diag::known_packets())
            cout << std::format("{:<55} {:>#12x} {:>5}  {}", entry.name, entry.address,
                                entry.request_is_rtr ? "yes" : "no", entry.description)
                 << endl;
    }

    // Resolves the CLI options down to a single target address, ignoring the
    // const bitfield member on standard_address_t by only ever constructing
    // fresh instances (it's not copy-assignable).
    addressing::flagged_address_t resolve_target(const can_diag::cli_options_t& opts)
    {
        uint8_t system = 0, subsystem = 0, device = 0, group = 0, parameter = 0;
        bool rtr = false;

        if (opts.preset_name)
        {
            const auto preset = can_diag::find_preset_by_name(*opts.preset_name);
            if (!preset)
                throw std::invalid_argument("Unknown preset: '" + *opts.preset_name +
                                            "' (see --list-presets)");
            const addressing::standard_address_t decoded(preset->address);
            system = decoded.system;
            subsystem = decoded.subsystem;
            device = decoded.device;
            group = decoded.group;
            parameter = decoded.parameter;
            rtr = preset->request_is_rtr;
        }
        else if (opts.explicit_id)
        {
            const addressing::standard_address_t decoded(*opts.explicit_id);
            system = decoded.system;
            subsystem = decoded.subsystem;
            device = decoded.device;
            group = decoded.group;
            parameter = decoded.parameter;
        }

        if (opts.system)
            system = *opts.system;
        if (opts.subsystem)
            subsystem = *opts.subsystem;
        if (opts.device)
            device = *opts.device;
        if (opts.group)
            group = *opts.group;
        if (opts.parameter)
            parameter = *opts.parameter;
        if (opts.rtr_override)
            rtr = *opts.rtr_override;

        const addressing::standard_address_t fields(system, subsystem, device, group, parameter);
        return addressing::flagged_address_t(static_cast<addressing::raw_address_t>(fields), rtr);
    }
}  // namespace

int main(int argc, const char** argv)
{
    can_diag::cli_options_t opts;
    try
    {
        opts = can_diag::parse_args(argc, argv);
    }
    catch (const std::exception& e)
    {
        cout << "Error: " << e.what() << endl;
        can_diag::print_usage();
        return 1;
    }

    if (opts.help)
    {
        can_diag::print_usage();
        return 0;
    }

    if (opts.list_presets)
    {
        print_presets();
        return 0;
    }

    try
    {
        const addressing::flagged_address_t target_address = resolve_target(opts);
        const addressing::raw_address_t target_raw = target_address.address;

        RawCanInterface can_interface(opts.iface);
        cout << "Opened CAN interface: " << opts.iface << endl;

        if (!opts.listen)
        {
            const std::vector<uint8_t> tx_data = opts.data.value_or(std::vector<uint8_t>{});
            const Packet tx_packet(target_address, tx_data);
            print_address(target_address, true);
            print_data(tx_data);
            can_interface.transmit(tx_packet);
        }
        else
        {
            print_address(target_address, true);
            cout << "  (listening only, not transmitting)" << endl;
        }

        if (opts.timeout_ms == 0)
        {
            cout << "Not waiting for a response (timeout=0)." << endl;
            return 0;
        }

        cout << "Waiting up to " << opts.timeout_ms << "ms for a response..." << endl;

        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(opts.timeout_ms);
        while (std::chrono::steady_clock::now() < deadline)
        {
            const auto received = can_interface.receive(false);
            if (!received)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }

            if (received->get_address().address != target_raw)
                continue;

            print_address(received->get_address(), false);

            const auto matched_preset = opts.raw
                                            ? std::nullopt
                                            : can_diag::find_preset_by_address(target_raw);
            if (matched_preset && matched_preset->decode)
                cout << "  decoded (" << matched_preset->name
                     << "): " << matched_preset->decode(received->get_data()) << endl;
            print_data(received->get_data());

            return 0;
        }

        cout << "No response within " << opts.timeout_ms << "ms." << endl;
        return 1;
    }
    catch (const std::exception& e)
    {
        cout << "Error: " << e.what() << endl;
        return 1;
    }
}
