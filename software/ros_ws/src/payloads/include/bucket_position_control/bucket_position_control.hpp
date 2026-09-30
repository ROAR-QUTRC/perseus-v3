#pragma once

#include <array>
#include <hi_can_raw.hpp>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

/// @brief Drives the excavation bucket to named preset positions over CAN.
///
/// Positions are loaded from parameters (see config/bucket_positions.yaml),
/// each giving a lift, tilt and jaws angle in degrees. Publishing a position's
/// name on `bucket_position` sends one SET_POSITION frame per enabled bank.
///
/// Like BucketDriver this bypasses ros2_control. The firmware picks its control
/// mode from whichever command it saw last, so this node must not run alongside
/// bucket_driver or the ros2_control stack.
class BucketPositionControl : public rclcpp::Node
{
public:
    explicit BucketPositionControl(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~BucketPositionControl() override;

private:
    /// @brief Number of actuator banks - lift, tilt and jaws, in that order.
    static constexpr size_t ACTUATOR_COUNT = 3;

    void _position_callback(const std_msgs::msg::String::SharedPtr msg);

    /// @brief Named positions, in degrees, ordered lift, tilt, jaws.
    std::map<std::string, std::array<double, ACTUATOR_COUNT>> _positions;
    /// @brief Which banks are actually sent SET_POSITION frames. Positions
    /// define every bank, but only these are commanded.
    std::array<bool, ACTUATOR_COUNT> _enabled{};

    hi_can::RawCanInterface _can_interface;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _position_subscription;
};
