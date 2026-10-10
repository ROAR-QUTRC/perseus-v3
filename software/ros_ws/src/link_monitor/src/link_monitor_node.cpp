/// @file link_monitor_node.cpp
/// @brief Publishes throughput and reachability of the link to the rover as
/// interfaces/LinkHealth.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <fstream>
#include <interfaces/msg/device_link.hpp>
#include <interfaces/msg/link_health.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <string>
#include <thread>
#include <vector>

namespace link_monitor
{
    namespace
    {

        using DeviceLink = interfaces::msg::DeviceLink;
        using LinkHealth = interfaces::msg::LinkHealth;

        /// @brief Runs a command and returns what it wrote to stdout.
        std::string run_command(const std::string& command)
        {
            std::string output;
            // NOLINTNEXTLINE(cert-env33-c): the command is built from configured IPs only
            FILE* pipe = popen(command.c_str(), "r");
            if (pipe == nullptr)
            {
                return output;
            }
            char buffer[256];
            while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
            {
                output += buffer;
            }
            pclose(pipe);
            return output;
        }

        /// @brief True if the string is a plain dotted IPv4 address. Guards run_command.
        bool is_ipv4(const std::string& text)
        {
            static const std::regex pattern(R"(^\d{1,3}(\.\d{1,3}){3}$)");
            return std::regex_match(text, pattern);
        }

        /// @brief Interface the kernel would send traffic to this address over.
        std::optional<std::string> interface_for(const std::string& ip)
        {
            if (!is_ipv4(ip))
            {
                return std::nullopt;
            }
            const std::string output = run_command("ip -o route get " + ip + " 2>/dev/null");
            std::smatch match;
            if (std::regex_search(output, match, std::regex(R"(\bdev\s+(\S+))")))
            {
                return match[1].str();
            }
            return std::nullopt;
        }

        /// @brief Reads one counter from /sys/class/net/<interface>/statistics.
        std::optional<std::uint64_t> read_counter(const std::string& interface,
                                                  const std::string& name)
        {
            std::ifstream file("/sys/class/net/" + interface + "/statistics/" + name);
            std::uint64_t value = 0;
            if (file >> value)
            {
                return value;
            }
            return std::nullopt;
        }

        /// @brief Negotiated speed in Mbit/s, or -1 if the kernel does not report one.
        float read_speed_mbps(const std::string& interface)
        {
            std::ifstream file("/sys/class/net/" + interface + "/speed");
            std::int64_t speed = -1;
            if (file >> speed && speed > 0)
            {
                return static_cast<float>(speed);
            }
            return -1.0F;
        }

        /// @brief One ICMP echo through the system ping, which has the privileges a plain
        /// process lacks. Returns the round trip in milliseconds, or nothing if unanswered.
        std::optional<double> ping_once(const std::string& ip)
        {
            const std::string output =
                run_command("ping -n -c 1 -W 1 " + ip + " 2>/dev/null");
            std::smatch match;
            if (std::regex_search(output, match, std::regex(R"(time[=<]([0-9.]+)\s*ms)")))
            {
                return std::stod(match[1].str());
            }
            return std::nullopt;
        }

        /// @brief Rolling record of the last probes to one device.
        class PingHistory
        {
        public:
            explicit PingHistory(std::size_t window)
                : _window(window)
            {
            }

            void add(std::optional<double> rtt_ms)
            {
                const std::lock_guard<std::mutex> lock(_mutex);
                _samples.push_back(rtt_ms);
                if (_samples.size() > _window)
                {
                    _samples.pop_front();
                }
                _consecutive_misses = rtt_ms ? 0 : _consecutive_misses + 1;
                _probed = true;
            }

            struct Summary
            {
                bool probed{false};
                int consecutive_misses{0};
                float rtt_ms{-1.0F};
                float jitter_ms{-1.0F};
                float loss_percent{0.0F};
            };

            Summary summarise() const
            {
                const std::lock_guard<std::mutex> lock(_mutex);
                Summary summary;
                summary.probed = _probed;
                summary.consecutive_misses = _consecutive_misses;
                if (_samples.empty())
                {
                    return summary;
                }
                std::vector<double> replies;
                for (const auto& sample : _samples)
                {
                    if (sample)
                    {
                        replies.push_back(*sample);
                    }
                }
                summary.loss_percent = static_cast<float>(
                    100.0 * static_cast<double>(_samples.size() - replies.size()) /
                    static_cast<double>(_samples.size()));
                if (!replies.empty())
                {
                    double sum = 0.0;
                    for (const double rtt : replies)
                    {
                        sum += rtt;
                    }
                    const double mean = sum / static_cast<double>(replies.size());
                    summary.rtt_ms = static_cast<float>(mean);
                    if (replies.size() > 1)
                    {
                        double squares = 0.0;
                        for (const double rtt : replies)
                        {
                            squares += (rtt - mean) * (rtt - mean);
                        }
                        summary.jitter_ms = static_cast<float>(
                            std::sqrt(squares / static_cast<double>(replies.size())));
                    }
                }
                return summary;
            }

        private:
            mutable std::mutex _mutex;
            std::deque<std::optional<double>> _samples;
            std::size_t _window;
            int _consecutive_misses{0};
            bool _probed{false};
        };

        struct Device
        {
            std::string name;
            std::string ip;
            std::unique_ptr<PingHistory> history;
        };

    }  // namespace

    /// @brief Probes each configured device and samples the routed interface's counters,
    /// publishing one LinkHealth per period.
    class LinkMonitorNode : public rclcpp::Node
    {
    public:
        LinkMonitorNode()
            : rclcpp::Node("link_monitor")
        {
            const auto names = declare_parameter("device_names", std::vector<std::string>{});
            const auto ips = declare_parameter("device_ips", std::vector<std::string>{});
            _interface_override = declare_parameter("interface", std::string{});
            const double publish_rate_hz = declare_parameter("publish_rate_hz", 1.0);
            _detect_period = std::chrono::duration<double>(
                declare_parameter("interface_check_period_sec", 3.0));
            _rtt_warn_ms = declare_parameter("rtt_warn_ms", 100.0);
            _loss_warn_percent = declare_parameter("loss_warn_percent", 10.0);
            _down_after_misses = static_cast<int>(declare_parameter("down_after_misses", 3));
            _utilisation_warn = declare_parameter("utilisation_warn", 0.8);
            const auto window = static_cast<std::size_t>(
                std::max<std::int64_t>(declare_parameter("ping_window", 10), 1));

            if (names.size() != ips.size())
            {
                RCLCPP_ERROR(get_logger(),
                             "device_names (%zu) and device_ips (%zu) differ in length",
                             names.size(), ips.size());
            }
            for (std::size_t i = 0; i < std::min(names.size(), ips.size()); ++i)
            {
                if (!is_ipv4(ips[i]))
                {
                    RCLCPP_ERROR(get_logger(), "Skipping %s: '%s' is not an IPv4 address",
                                 names[i].c_str(), ips[i].c_str());
                    continue;
                }
                _devices.push_back({names[i], ips[i], std::make_unique<PingHistory>(window)});
            }
            if (_devices.empty())
            {
                RCLCPP_WARN(get_logger(), "No devices to monitor; only throughput is reported");
            }

            _publisher = create_publisher<LinkHealth>("link_health", rclcpp::QoS(1));

            for (auto& device : _devices)
            {
                _threads.emplace_back([this, &device]
                                      { _ping_loop(device); });
            }

            const auto period = std::chrono::duration<double>(
                1.0 / std::max(publish_rate_hz, 0.01));
            _timer = create_wall_timer(period, [this]
                                       { _publish(); });
        }

        ~LinkMonitorNode() override
        {
            _running = false;
            for (auto& thread : _threads)
            {
                thread.join();
            }
        }

        LinkMonitorNode(const LinkMonitorNode&) = delete;
        LinkMonitorNode& operator=(const LinkMonitorNode&) = delete;

    private:
        /// @brief Probes one device about once a second until shutdown.
        void _ping_loop(Device& device)
        {
            while (_running)
            {
                const auto started = std::chrono::steady_clock::now();
                device.history->add(ping_once(device.ip));
                // ping returns at once on a reply, so pace the probes; sleep in short
                // steps so shutdown is not held up.
                while (_running && std::chrono::steady_clock::now() - started <
                                       std::chrono::seconds(1))
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            }
        }

        /// @brief Picks the interface to read counters from, at most once per detect period.
        void _update_interface()
        {
            const auto now_time = std::chrono::steady_clock::now();
            if (_last_detect && now_time - *_last_detect < _detect_period)
            {
                return;
            }
            _last_detect = now_time;

            std::string detected;
            if (!_interface_override.empty())
            {
                detected = _interface_override;
            }
            else
            {
                for (const auto& device : _devices)
                {
                    if (const auto found = interface_for(device.ip))
                    {
                        detected = *found;
                        break;
                    }
                }
            }
            if (detected != _interface)
            {
                if (!detected.empty())
                {
                    RCLCPP_INFO(get_logger(), "Rover link is on interface %s", detected.c_str());
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "No route to any rover device");
                }
                _interface = detected;
                // A baseline from another interface would make the next delta meaningless.
                _previous_counters.reset();
            }
        }

        void _publish()
        {
            _update_interface();

            LinkHealth message;
            message.header.stamp = now();
            message.interface = _interface;
            message.rx_bytes_per_sec = -1.0;
            message.tx_bytes_per_sec = -1.0;
            message.link_speed_mbps = -1.0F;

            if (!_interface.empty())
            {
                message.link_speed_mbps = read_speed_mbps(_interface);
                const auto rx = read_counter(_interface, "rx_bytes");
                const auto tx = read_counter(_interface, "tx_bytes");
                const auto sampled = std::chrono::steady_clock::now();
                if (rx && tx)
                {
                    if (_previous_counters)
                    {
                        const double seconds =
                            std::chrono::duration<double>(sampled - _previous_counters->time)
                                .count();
                        if (seconds > 0.0 && *rx >= _previous_counters->rx &&
                            *tx >= _previous_counters->tx)
                        {
                            message.rx_bytes_per_sec =
                                static_cast<double>(*rx - _previous_counters->rx) / seconds;
                            message.tx_bytes_per_sec =
                                static_cast<double>(*tx - _previous_counters->tx) / seconds;
                        }
                    }
                    _previous_counters = Counters{*rx, *tx, sampled};
                }
            }

            std::uint8_t worst = _interface.empty() ? LinkHealth::STATUS_ERROR
                                                    : LinkHealth::STATUS_OK;
            if (message.link_speed_mbps > 0.0F && message.rx_bytes_per_sec >= 0.0)
            {
                const double capacity_bytes = message.link_speed_mbps * 125000.0;
                const double busiest =
                    std::max(message.rx_bytes_per_sec, message.tx_bytes_per_sec);
                if (busiest > _utilisation_warn * capacity_bytes)
                {
                    worst = std::max<std::uint8_t>(worst, LinkHealth::STATUS_DEGRADED);
                }
            }

            std::size_t down_count = 0;
            for (const auto& device : _devices)
            {
                const auto summary = device.history->summarise();
                DeviceLink link;
                link.name = device.name;
                link.ip = device.ip;
                link.rtt_ms = summary.rtt_ms;
                link.jitter_ms = summary.jitter_ms;
                link.loss_percent = summary.loss_percent;

                if (!summary.probed || summary.consecutive_misses >= _down_after_misses)
                {
                    // Not yet probed counts as down: better a red dot for a second at
                    // startup than a green one that has not been earned.
                    link.status = DeviceLink::STATUS_DOWN;
                    ++down_count;
                }
                else if (summary.loss_percent > _loss_warn_percent ||
                         summary.rtt_ms > _rtt_warn_ms)
                {
                    link.status = DeviceLink::STATUS_DEGRADED;
                }
                else
                {
                    link.status = DeviceLink::STATUS_OK;
                }
                message.devices.push_back(link);
            }

            if (!_devices.empty() && down_count == _devices.size())
            {
                worst = LinkHealth::STATUS_ERROR;
            }
            else if (down_count > 0)
            {
                worst = std::max<std::uint8_t>(worst, LinkHealth::STATUS_DEGRADED);
            }
            for (const auto& link : message.devices)
            {
                if (link.status == DeviceLink::STATUS_DEGRADED)
                {
                    worst = std::max<std::uint8_t>(worst, LinkHealth::STATUS_DEGRADED);
                }
            }
            message.overall_status = worst;

            _publisher->publish(message);
        }

        struct Counters
        {
            std::uint64_t rx;
            std::uint64_t tx;
            std::chrono::steady_clock::time_point time;
        };

        std::vector<Device> _devices;
        std::vector<std::thread> _threads;
        std::atomic<bool> _running{true};

        std::string _interface_override;
        std::string _interface;
        std::optional<std::chrono::steady_clock::time_point> _last_detect;
        std::chrono::duration<double> _detect_period{3.0};
        std::optional<Counters> _previous_counters;

        double _rtt_warn_ms{100.0};
        double _loss_warn_percent{10.0};
        int _down_after_misses{3};
        double _utilisation_warn{0.8};

        rclcpp::Publisher<LinkHealth>::SharedPtr _publisher;
        rclcpp::TimerBase::SharedPtr _timer;
    };

}  // namespace link_monitor

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<link_monitor::LinkMonitorNode>());
    rclcpp::shutdown();
    return 0;
}
