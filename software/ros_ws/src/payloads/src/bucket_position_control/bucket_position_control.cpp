#include "bucket_position_control/bucket_position_control.hpp"

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

    /// @brief Banks in the order positions are stored.
    constexpr std::array<bucket_addr::bank_group, 3> BANKS{
        bucket_addr::bank_group::LIFT,
        bucket_addr::bank_group::TILT,
        bucket_addr::bank_group::JAWS,
    };

    /// @brief Bank names, as used in parameter names and log messages.
    constexpr std::array<const char*, 3> BANK_NAMES{"lift", "tilt", "jaws"};

    /// @brief AS5600 counts per degree - the firmware compares SET_POSITION
    /// directly against raw encoder counts (4096 per revolution).
    constexpr double COUNTS_PER_DEGREE = 4096.0 / 360.0;

    addressing::flagged_address_t bank_address(
        const bucket_addr::bank_group& bank,
        const bucket_addr::bank_parameter& parameter)
    {
        return static_cast<addressing::flagged_address_t>(
            addressing::standard_address_t{
                DEVICE_ADDRESS,
                static_cast<uint8_t>(bank),
                static_cast<uint8_t>(parameter)});
    }

    /// @brief Converts degrees to encoder counts, throwing if it won't fit.
    int16_t degrees_to_counts(double degrees)
    {
        const double counts = std::round(degrees * COUNTS_PER_DEGREE);
        if (!std::isfinite(counts) || counts < INT16_MIN || counts > INT16_MAX)
            throw std::out_of_range("position " + std::to_string(degrees) +
                                    " deg does not fit in an int16 of counts");
        return static_cast<int16_t>(counts);
    }
}  // namespace

BucketPositionControl::BucketPositionControl(const rclcpp::NodeOptions& options)
    : Node("bucket_position_control", options)
{
    _can_interface =
        hi_can::RawCanInterface(this->declare_parameter("can_bus", "can0"));

    const auto enabled_axes = this->declare_parameter(
        "enabled_axes", std::vector<std::string>{"lift"});
    for (const auto& axis : enabled_axes)
    {
        const auto it = std::find_if(BANK_NAMES.begin(), BANK_NAMES.end(),
                                     [&](const char* name)
                                     { return axis == name; });
        if (it == BANK_NAMES.end())
            throw std::invalid_argument("Unknown axis in enabled_axes: " + axis);
        _enabled[static_cast<size_t>(it - BANK_NAMES.begin())] = true;
    }

    const auto position_names = this->declare_parameter(
        "position_names", std::vector<std::string>{});
    if (position_names.empty())
        throw std::invalid_argument("position_names is empty - no positions defined");

    for (const auto& name : position_names)
    {
        std::array<double, ACTUATOR_COUNT> degrees{};
        for (size_t i = 0; i < ACTUATOR_COUNT; i++)
        {
            // No default, so a position missing an axis fails at startup
            // rather than silently commanding zero.
            const std::string param = "positions." + name + "." + BANK_NAMES[i];
            degrees[i] = this->declare_parameter<double>(param);
            // Validate now so a bad value can't throw mid-command later.
            degrees_to_counts(degrees[i]);
        }
        _positions[name] = degrees;
    }

    _position_subscription = this->create_subscription<std_msgs::msg::String>(
        "bucket_position", 10,
        std::bind(&BucketPositionControl::_position_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Bucket position control initialized - %zu positions loaded",
                _positions.size());
}

BucketPositionControl::~BucketPositionControl()
{
    // SET_POSITION has no firmware watchdog, so the banks would keep holding
    // the last setpoint forever. A zero SET_SPEED stops them and drops the
    // firmware out of position mode.
    try
    {
        for (size_t i = 0; i < ACTUATOR_COUNT; i++)
        {
            if (!_enabled[i])
                continue;
            _can_interface.transmit(
                Packet(bank_address(BANKS[i], bucket_addr::bank_parameter::SET_SPEED),
                       bucket_param::speed_t{0}.serialize_data()));
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to stop actuators on shutdown: %s",
                     e.what());
    }
}

void BucketPositionControl::_position_callback(
    const std_msgs::msg::String::SharedPtr msg)
{
    const auto it = _positions.find(msg->data);
    if (it == _positions.end())
    {
        std::string valid;
        for (const auto& [name, _] : _positions)
            valid += (valid.empty() ? "" : ", ") + name;
        RCLCPP_WARN(this->get_logger(), "Unknown position '%s' - valid: %s",
                    msg->data.c_str(), valid.c_str());
        return;
    }

    for (size_t i = 0; i < ACTUATOR_COUNT; i++)
    {
        if (!_enabled[i])
            continue;

        const int16_t counts = degrees_to_counts(it->second[i]);
        RCLCPP_INFO(this->get_logger(), "%s: %s -> %.2f deg (%d counts)",
                    it->first.c_str(), BANK_NAMES[i], it->second[i], counts);

        _can_interface.transmit(
            Packet(bank_address(BANKS[i], bucket_addr::bank_parameter::SET_POSITION),
                   bucket_param::position_t{counts}.serialize_data()));
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    int exit_code = 0;
    try
    {
        auto node = std::make_shared<BucketPositionControl>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting bucket position control node");
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"),
                     "Error running bucket position control: %s", e.what());
        exit_code = 1;
    }

    // Has to run on the failure path too - returning straight out of the catch
    // leaves the context alive into static destruction, which segfaults on exit.
    rclcpp::shutdown();
    return exit_code;
}
