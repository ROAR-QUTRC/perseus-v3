#pragma once

#include <actuator_msgs/msg/actuators.hpp>
#include <array>
#include <chrono>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <hi_can_raw.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

/// @brief Teleop driver for the excavation bucket - joystick speeds to CAN.
///
/// Converts the `Actuators` messages published by teleop's generic_controller
/// into bank-level SET_SPEED frames. This is the open-loop manual path and is
/// deliberately outside ros2_control: the autonomy path (BucketHardware) drives
/// the same banks with SET_POSITION instead.
///
/// It runs alongside the ros2_control stack as the operator's override. The
/// firmware picks its control mode from whichever command it saw last, so the two
/// must never transmit at once: until a stick passes override_deadband, only zeros
/// go out (which the firmware ignores in position mode). The first input past it
/// deactivates whichever of override_controllers are active, and only once that is
/// confirmed (or OVERRIDE_SWITCH_TIMEOUT passes - the operator always wins) do real
/// speeds go out. Autonomy gets the bucket back only through /bucket/rearm.
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

    /// @brief How long a takeover waits for controller_manager to confirm the
    /// command controllers are off before speeds go out regardless. Keeps the
    /// operator in charge even with controller_manager gone or wedged.
    static constexpr auto OVERRIDE_SWITCH_TIMEOUT = std::chrono::milliseconds(200);

    using ListControllers = controller_manager_msgs::srv::ListControllers;
    using SwitchController = controller_manager_msgs::srv::SwitchController;
    using Trigger = std_srvs::srv::Trigger;

    void _actuator_callback(const actuator_msgs::msg::Actuators::SharedPtr msg);
    void _transmit_callback();

    /// @brief Whether any of these speeds is a deliberate stick input.
    bool _above_deadband(const std::array<double, ACTUATOR_COUNT>& speeds) const;

    /// @brief Starts a takeover: deactivate whichever override_controllers are active.
    void _begin_override();
    /// @brief Marks the takeover done and tells everyone listening.
    void _set_override(bool active);

    /// @brief /bucket/rearm: hand the bucket back to rearm_controller.
    void _rearm_callback(rclcpp::Service<Trigger>::SharedPtr service,
                         std::shared_ptr<rmw_request_id_t> header,
                         std::shared_ptr<Trigger::Request> request);

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

    /// @brief Fraction of _max_actuator_speed a stick must pass to take over.
    double _override_deadband = 0.1;
    /// @brief The bucket's command controllers, deactivated by a takeover.
    std::vector<std::string> _override_controllers;
    /// @brief The controller /bucket/rearm activates.
    std::string _rearm_controller;

    /// @brief Whether the operator has the bucket: speeds go out as commanded.
    bool _override = false;
    /// @brief When a takeover started, while its switch is still unconfirmed.
    std::optional<rclcpp::Time> _override_requested;

    hi_can::RawCanInterface _can_interface;

    rclcpp::Subscription<actuator_msgs::msg::Actuators>::SharedPtr
        _actuator_subscription;
    rclcpp::TimerBase::SharedPtr _transmit_timer;
    rclcpp::Client<ListControllers>::SharedPtr _list_client;
    rclcpp::Client<SwitchController>::SharedPtr _switch_client;
    rclcpp::Service<Trigger>::SharedPtr _rearm_service;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _override_publisher;
};
