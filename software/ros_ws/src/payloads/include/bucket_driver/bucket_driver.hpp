#pragma once

#include <actuator_msgs/msg/actuators.hpp>
#include <array>
#include <chrono>
#include <hi_can_raw.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>

/// @brief Teleop driver for the excavation bucket - joystick speeds to CAN.
///
/// Converts the `Actuators` messages published by teleop's generic_controller
/// into bank-level SET_SPEED frames. This is the open-loop manual path and is
/// deliberately outside ros2_control: the autonomy path (BucketHardware) drives
/// the same banks with SET_POSITION instead.
///
/// The firmware picks its control mode from whichever command it saw last, so
/// this node and the ros2_control stack must never run at the same time.
class BucketDriver : public rclcpp::Node
{
public:
    explicit BucketDriver(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~BucketDriver() override;

private:
    /// @brief Number of actuator banks - lift, tilt and jaws, in that order.
    /// Matches both the CAN bank groups and the velocity layout teleop sends.
    static constexpr size_t ACTUATOR_COUNT = 3;

    /// @brief How often speeds are pushed to CAN.
    ///
    /// Transmission is on a timer rather than straight out of the subscription
    /// callback so that our frame rate doesn't inherit whatever rate the
    /// joystick pipeline happens to publish at. The firmware stops the banks if
    /// it doesn't see a SET_SPEED for 200ms, so this has to stay well inside
    /// that regardless of what upstream is doing.
    static constexpr auto TRANSMIT_INTERVAL = std::chrono::milliseconds(50);

    /// @brief How long the last command is held before being zeroed.
    ///
    /// Shorter than the firmware's 200ms watchdog on purpose, so a stalled
    /// publisher is caught here - with an explanatory log - rather than by the
    /// firmware timing out silently.
    static constexpr auto ACTUATOR_TIMEOUT = std::chrono::milliseconds(100);

    void _actuator_callback(const actuator_msgs::msg::Actuators::SharedPtr msg);
    void _transmit_callback();

    void _write_speeds(const std::array<double, ACTUATOR_COUNT>& speeds);

    /// @brief Velocity which maps to full duty cycle. Tunable so the bucket can
    /// be de-rated for bring-up without a rebuild.
    double _max_actuator_speed = 0.1;

    /// @brief Last commanded speeds, resent every TRANSMIT_INTERVAL.
    std::array<double, ACTUATOR_COUNT> _speeds{};
    /// @brief When the last valid command arrived - unset until the first one.
    std::optional<rclcpp::Time> _last_command_time;
    /// @brief Whether _speeds has already been zeroed by the timeout, so the
    /// warning is logged once per stall rather than every cycle.
    bool _timed_out = true;

    hi_can::RawCanInterface _can_interface;

    rclcpp::Subscription<actuator_msgs::msg::Actuators>::SharedPtr
        _actuator_subscription;
    rclcpp::TimerBase::SharedPtr _transmit_timer;
};
