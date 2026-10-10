/// @file system_monitor_node.cpp
/// @brief Publishes CPU, memory and temperature of the host as interfaces/MachineHealth.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <interfaces/msg/machine_health.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace system_monitor
{
    namespace
    {

        /// @brief Cumulative jiffies of the aggregate "cpu" line of /proc/stat.
        struct CpuSample
        {
            std::uint64_t busy{0};
            std::uint64_t total{0};
        };

        /// @brief Reads the aggregate CPU counters, or nothing if /proc/stat is unreadable.
        std::optional<CpuSample> read_cpu_sample()
        {
            std::ifstream stat("/proc/stat");
            std::string label;
            // Fields after the label: user nice system idle iowait irq softirq steal.
            std::uint64_t user{0}, nice{0}, system{0}, idle{0}, iowait{0}, irq{0},
                softirq{0}, steal{0};
            if (!(stat >> label >> user >> nice >> system >> idle >> iowait >> irq >>
                  softirq >> steal) ||
                label != "cpu")
            {
                return std::nullopt;
            }
            // Time waiting on I/O is the CPU being idle, as top reports it.
            const std::uint64_t idle_time = idle + iowait;
            const std::uint64_t busy_time = user + nice + system + irq + softirq + steal;
            return CpuSample{busy_time, busy_time + idle_time};
        }

        /// @brief Total and available physical memory, in bytes.
        struct MemorySample
        {
            std::uint64_t total_bytes{0};
            std::uint64_t available_bytes{0};
        };

        /// @brief Reads MemTotal and MemAvailable from /proc/meminfo.
        std::optional<MemorySample> read_memory_sample()
        {
            std::ifstream meminfo("/proc/meminfo");
            MemorySample sample;
            bool have_total = false;
            bool have_available = false;
            std::string key;
            std::uint64_t value_kib = 0;
            std::string unit;
            while (meminfo >> key >> value_kib)
            {
                std::getline(meminfo, unit);  // discard the "kB" and the newline
                if (key == "MemTotal:")
                {
                    sample.total_bytes = value_kib * 1024;
                    have_total = true;
                }
                else if (key == "MemAvailable:")
                {
                    sample.available_bytes = value_kib * 1024;
                    have_available = true;
                }
                if (have_total && have_available)
                {
                    return sample;
                }
            }
            return std::nullopt;
        }

        /// @brief Hottest thermal zone in degrees Celsius, or -1 if there is none.
        float read_max_temperature_c()
        {
            namespace fs = std::filesystem;
            float hottest = -1.0F;
            std::error_code error;
            for (const auto& entry :
                 fs::directory_iterator("/sys/class/thermal", error))
            {
                if (entry.path().filename().string().rfind("thermal_zone", 0) != 0)
                {
                    continue;
                }
                std::ifstream file(entry.path() / "temp");
                std::int64_t millidegrees = 0;
                if (file >> millidegrees)
                {
                    hottest = std::max(hottest, static_cast<float>(millidegrees) / 1000.0F);
                }
            }
            return hottest;
        }

    }  // namespace

    /// @brief Samples the host once per period and publishes the result.
    ///
    /// The node is meant to be launched under a namespace named after the machine, which
    /// is what keeps several machines' topics apart. The topic is relative, so it lands
    /// on /<namespace>/system_health.
    class SystemMonitorNode : public rclcpp::Node
    {
    public:
        SystemMonitorNode() : rclcpp::Node("system_monitor")
        {
            const double publish_rate_hz = declare_parameter("publish_rate_hz", 1.0);
            _device_name = declare_parameter("device_name", std::string{});
            if (_device_name.empty())
            {
                // Strip the leading slash of the namespace, so the name matches the topic
                // prefix even when it was not passed in explicitly.
                _device_name = std::string(get_namespace()).substr(1);
            }

            _publisher = create_publisher<interfaces::msg::MachineHealth>(
                "system_health", rclcpp::QoS(1));

            _previous_cpu = read_cpu_sample();

            const auto period = std::chrono::duration<double>(
                1.0 / std::max(publish_rate_hz, 0.01));
            _timer = create_wall_timer(period, [this] { _publish(); });
        }

    private:
        void _publish()
        {
            interfaces::msg::MachineHealth message;
            message.header.stamp = now();
            message.device_name = _device_name;

            message.cpu_percent = -1.0F;
            const auto cpu = read_cpu_sample();
            if (cpu && _previous_cpu && cpu->total > _previous_cpu->total)
            {
                const auto busy = static_cast<double>(cpu->busy - _previous_cpu->busy);
                const auto total = static_cast<double>(cpu->total - _previous_cpu->total);
                message.cpu_percent = static_cast<float>(100.0 * busy / total);
            }
            _previous_cpu = cpu;

            message.memory_percent = -1.0F;
            if (const auto memory = read_memory_sample(); memory && memory->total_bytes > 0)
            {
                const std::uint64_t used = memory->total_bytes - memory->available_bytes;
                message.memory_total_bytes = memory->total_bytes;
                message.memory_used_bytes = used;
                message.memory_percent = static_cast<float>(
                    100.0 * static_cast<double>(used) /
                    static_cast<double>(memory->total_bytes));
            }

            message.temperature_c = read_max_temperature_c();

            _publisher->publish(message);
        }

        std::string _device_name;
        std::optional<CpuSample> _previous_cpu;
        rclcpp::Publisher<interfaces::msg::MachineHealth>::SharedPtr _publisher;
        rclcpp::TimerBase::SharedPtr _timer;
    };

}  // namespace system_monitor

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<system_monitor::SystemMonitorNode>());
    rclcpp::shutdown();
    return 0;
}
