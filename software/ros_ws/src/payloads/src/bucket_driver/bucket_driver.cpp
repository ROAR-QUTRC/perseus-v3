#include "bucket_driver/bucket_driver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <hi_can_address.hpp>
#include <hi_can_parameter.hpp>
#include <stdexcept>

namespace
{
    using namespace hi_can;  // NOLINT

    namespace bucket_addr = addressing::excavation::bucket::controller;
    namespace bucket_param = parameters::excavation::bucket::controller;

    /// @brief Address of the bucket controller board itself.
    const addressing::standard_address_t DEVICE_ADDRESS{
        addressing::excavation::SYSTEM_ID,
        addressing::excavation::bucket::SUBSYSTEM_ID,
        bucket_addr::DEVICE_ID};

    /// @brief Banks in the order teleop sends their velocities.
    constexpr std::array<bucket_addr::bank_group, 3> BANKS{
        bucket_addr::bank_group::LIFT,
        bucket_addr::bank_group::TILT,
        bucket_addr::bank_group::JAWS,
    };

    /// @brief Human-readable bank names, for log messages only.
    constexpr std::array<const char*, 3> BANK_NAMES{"lift", "tilt", "jaws"};

    addressing::flagged_address_t set_speed_address(
        const bucket_addr::bank_group& bank)
    {
        return static_cast<addressing::flagged_address_t>(
            addressing::standard_address_t{
                DEVICE_ADDRESS,
                static_cast<uint8_t>(bank),
                static_cast<uint8_t>(bucket_addr::bank_parameter::SET_SPEED)});
    }
}  // namespace

BucketDriver::BucketDriver(const rclcpp::NodeOptions& options)
    : Node("bucket_driver", options)
{
    _max_actuator_speed = this->declare_parameter("max_actuator_speed", 0.1);
    if (_max_actuator_speed <= 0.0)
        throw std::invalid_argument("max_actuator_speed must be positive");

    _can_interface =
        hi_can::RawCanInterface(this->declare_parameter("can_bus", "can0"));

    _actuator_subscription =
        this->create_subscription<actuator_msgs::msg::Actuators>(
            "bucket_actuators", 10,
            std::bind(&BucketDriver::_actuator_callback, this,
                      std::placeholders::_1));

    _transmit_timer = this->create_timer(
        TRANSMIT_INTERVAL, std::bind(&BucketDriver::_transmit_callback, this));

    RCLCPP_INFO(this->get_logger(),
                "Bucket driver initialized - full duty at %.3f, transmitting every %ldms",
                _max_actuator_speed,
                static_cast<long>(TRANSMIT_INTERVAL.count()));
}

BucketDriver::~BucketDriver()
{
    // Stop the bucket on a clean shutdown rather than leaving it to the
    // firmware watchdog, which would keep driving for up to 200ms.
    try
    {
        _write_speeds({});
    }
    catch (const std::exception& e)
    {
        // Nothing useful to do about it this late - the firmware watchdog is
        // still behind us.
        RCLCPP_ERROR(this->get_logger(), "Failed to zero actuators on shutdown: %s",
                     e.what());
    }
}

void BucketDriver::_actuator_callback(
    const actuator_msgs::msg::Actuators::SharedPtr msg)
{
    if (msg->velocity.size() != ACTUATOR_COUNT)
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "Expected %zu actuator velocities (lift, tilt, jaws), got %zu - ignoring",
                              ACTUATOR_COUNT, msg->velocity.size());
        return;
    }

    // A non-finite velocity would make the clamp-and-cast in _write_speeds
    // undefined, so reject the whole message rather than commanding garbage.
    if (!std::all_of(msg->velocity.begin(),
                     msg->velocity.begin() + ACTUATOR_COUNT,
                     [](double v)
                     { return std::isfinite(v); }))
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "Actuator velocities contain a non-finite value - ignoring");
        return;
    }

    std::copy_n(msg->velocity.begin(), ACTUATOR_COUNT, _speeds.begin());
    _last_command_time = this->now();
    _timed_out = false;
}

void BucketDriver::_transmit_callback()
{
    const bool stale =
        !_last_command_time.has_value() ||
        ((this->now() - *_last_command_time) > rclcpp::Duration(ACTUATOR_TIMEOUT));

    if (stale && !_timed_out)
    {
        RCLCPP_WARN(this->get_logger(),
                    "No actuator command for %ldms - zeroing speeds",
                    static_cast<long>(ACTUATOR_TIMEOUT.count()));
        _speeds.fill(0.0);
        _timed_out = true;
    }

    // Sent unconditionally, including while zeroed: a steady stream of zeros
    // keeps the banks in a known state and means a gap on the bus unambiguously
    // signals that this node has died.
    _write_speeds(_speeds);
}

void BucketDriver::_write_speeds(
    const std::array<double, ACTUATOR_COUNT>& speeds)
{
    // speed_t's int16 is an open-loop duty cycle, not a physical speed - the
    // firmware maps the full int16 range across its PWM range. So pick the
    // velocity which should mean "flat out" and scale linearly to that.
    const double conversion_factor =
        static_cast<double>(INT16_MAX) / _max_actuator_speed;

    for (size_t i = 0; i < ACTUATOR_COUNT; i++)
    {
        const auto duty = static_cast<int16_t>(
            std::clamp(speeds[i] * conversion_factor,
                       static_cast<double>(INT16_MIN),
                       static_cast<double>(INT16_MAX)));

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "%s: %+.3f -> %+d", BANK_NAMES[i], speeds[i], duty);

        _can_interface.transmit(Packet(set_speed_address(BANKS[i]),
                                       bucket_param::speed_t{duty}.serialize_data()));
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    int exit_code = 0;
    try
    {
        auto node = std::make_shared<BucketDriver>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting bucket driver node");
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running bucket driver: %s",
                     e.what());
        exit_code = 1;
    }

    // Has to run on the failure path too - returning straight out of the catch
    // leaves the context alive into static destruction, which segfaults on exit.
    rclcpp::shutdown();
    return exit_code;
}
