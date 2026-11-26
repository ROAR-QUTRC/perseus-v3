#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <fstream>
#include <hi_can_raw.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace hi_can;
using std::cout, std::cerr, std::endl;

namespace
{
    constexpr size_t BUFFER_LINES = 15000;
    constexpr auto MAX_FLUSH_INTERVAL = std::chrono::seconds(2);

    std::atomic_bool g_running{true};

    void on_signal(int) { g_running = false; }

    void print_usage()
    {
        cout << "Usage: can_logger [--iface <name>] [--out <file>]\n"
                "  --iface <name>  CAN interface to log (default: any)\n"
                "  --out <file>    Output CSV file (default: ~/can_logs/can_log_<time>.csv)\n"
                "  --help          Show this message\n"
                "Every received frame is appended to one CSV file. Lines are buffered in\n"
                "memory ("
             << BUFFER_LINES
             << " lines) and written out when the buffer fills, every "
             << MAX_FLUSH_INTERVAL.count() << "s, and on exit (Ctrl+C / SIGTERM)."
             << endl;
    }

    std::filesystem::path default_output_path()
    {
        const char* home = std::getenv("HOME");
        const auto now = std::chrono::floor<std::chrono::seconds>(std::chrono::system_clock::now());
        return std::filesystem::path(home ? home : ".") / "can_logs" /
               std::format("can_log_{:%Y%m%d_%H%M%S}.csv", now);
    }

    void flush(std::ofstream& file, std::vector<std::string>& buffer)
    {
        for (const auto& line : buffer)
            file << line << '\n';
        file.flush();
        buffer.clear();
    }
}  // namespace

int main(int argc, const char** argv)
{
    std::string iface = "any";
    std::filesystem::path out_path = default_output_path();

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];     // log the first element of argv as the argument
        if (arg == "--help" || arg == "-h")  // if the argument is --help or -h, print usage and return 0
        {
            print_usage();
            return 0;
        }
        if (arg == "--iface" && i + 1 < argc)
            iface = argv[++i];  // log the second argument after --iface as the interface name
        else if (arg == "--out" && i + 1 < argc)
            out_path = argv[++i];
        else
        {
            cerr << "Unknown or incomplete argument: " << arg << endl;
            print_usage();
            return 1;
        }
    }

    try
    {
        if (out_path.has_parent_path())
            std::filesystem::create_directories(out_path.parent_path());
        std::ofstream file(out_path, std::ios::app);
        if (!file)
        {
            cerr << "Could not open " << out_path << " for writing" << endl;
            return 1;
        }

        RawCanInterface can_interface(iface);
        std::signal(SIGINT, on_signal);
        std::signal(SIGTERM, on_signal);
        cout << "Logging " << iface << " -> " << out_path << endl;

        file << "timestamp_us,id_hex,system,subsystem,device,group,parameter,rtr,extended,dlc,"
                "data_hex\n";

        std::vector<std::string> buffer;
        buffer.reserve(BUFFER_LINES);
        auto last_flush = std::chrono::steady_clock::now();
        size_t total = 0;

        while (g_running)
        {
            const auto received = can_interface.receive(false);
            if (received)
            {
                const auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(
                                        std::chrono::system_clock::now().time_since_epoch())
                                        .count();
                const auto& addr = received->get_address();
                const addressing::standard_address_t f(addr.address);
                const auto& data = received->get_data();

                std::string data_hex;
                for (const auto byte : data)
                    data_hex += std::format("{:02x}", byte);

                buffer.push_back(std::format(
                    "{},{:#010x},{:#04x},{:#04x},{:#04x},{:#04x},{:#04x},"
                    "{},{},{},{}",
                    now_us, addr.address, f.system, f.subsystem, f.device,
                    f.group, f.parameter, addr.is_rtr ? 1 : 0,
                    addr.is_extended ? 1 : 0, data.size(), data_hex));
                ++total;
            }
            else
                std::this_thread::sleep_for(std::chrono::milliseconds(1));

            const auto now = std::chrono::steady_clock::now();
            if (buffer.size() >= BUFFER_LINES ||
                (!buffer.empty() && now - last_flush >= MAX_FLUSH_INTERVAL))
            {
                flush(file, buffer);
                last_flush = now;
            }
        }

        flush(file, buffer);
        cout << "Stopped. Logged " << total << " frames to " << out_path << endl;
        return 0;
    }
    catch (const std::exception& e)
    {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }
}
