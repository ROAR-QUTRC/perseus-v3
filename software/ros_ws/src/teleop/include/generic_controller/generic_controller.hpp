#pragma once

#include <actuator_msgs/msg/actuators.hpp>
#include <chrono>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

class GenericController : public rclcpp::Node
{
public:
    explicit GenericController(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    friend class AxisParser;
    friend class EnableParser;

    class EnableParser;
    class AxisParser
    {
    public:
        AxisParser(GenericController& parent, const std::string& param_base_name,
                   bool has_enable = false);

        double get_value();

    private:
        std::vector<rclcpp::Parameter> _resolve_params();
        static std::vector<std::string>
        _get_param_names(const std::string& base_name);
        bool _parent_has_all_params(const std::string& follows_name);
        bool _parent_has_any_params(const std::string& follows_name);

        static constexpr size_t AXIS_IDX = 0;
        static constexpr size_t BUTTON_POSITIVE_IDX = 1;
        static constexpr size_t BUTTON_NEGATIVE_IDX = 2;
        static constexpr size_t SCALING_IDX = 3;
        static constexpr size_t FOLLOWS_IDX = 4;
        static constexpr size_t DEADBAND_POSITIVE_IDX = 5;
        static constexpr size_t DEADBAND_NEGATIVE_IDX = 6;
        static constexpr size_t TURBO_IDX = 7;

        std::map<std::string,
                 std::pair<int, rcl_interfaces::msg::ParameterDescriptor>>
            _int_param_map;
        std::map<std::string,
                 std::pair<double, rcl_interfaces::msg::ParameterDescriptor>>
            _double_param_map;
        std::map<std::string,
                 std::pair<std::string, rcl_interfaces::msg::ParameterDescriptor>>
            _string_param_map;

        GenericController& _parent;
        const std::string _param_base_name;
        bool _has_enable;
        double _last_value = 0.0;
    };

    class EnableParser
    {
    public:
        EnableParser(GenericController& parent, const std::string& param_base_name,
                     bool is_turbo = false);

        bool get_value();

    private:
        std::vector<rclcpp::Parameter> _resolve_params();
        static std::vector<std::string>
        _get_param_names(const std::string& base_name);

        static constexpr size_t FOLLOWS_IDX = 0;
        static constexpr size_t THRESHOLD_IDX = 1;
        static constexpr size_t IS_LT_IDX = 2;

        GenericController& _parent;
        const std::string _param_base_name;
    };
    /// @brief Who the controller is driving for.
    ///
    /// MANUAL publishes as it always has. AUTONOMY publishes nothing at all, so the
    /// gamepad cannot fight navigation or the bucket's trajectory controller; holding
    /// the drive deadman switches straight back to MANUAL, so the operator can always
    /// take over. ARMING is the short handover in between (/teleop/arm_autonomy).
    enum class Mode
    {
        MANUAL,
        ARMING,
        AUTONOMY,
    };

    using Trigger = std_srvs::srv::Trigger;

    void _joy_timeout_callback(void);
    void _joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    void _set_mode(Mode mode);
    /// @brief Publishes the remaining zero bursts, then nothing.
    void _publish_stop_bursts();

    /// @brief /teleop/arm_autonomy: stop, hand the bucket back, then go silent.
    void _arm_callback(rclcpp::Service<Trigger>::SharedPtr service,
                       std::shared_ptr<rmw_request_id_t> header,
                       std::shared_ptr<Trigger::Request> request);
    /// @brief /teleop/take_control: back to MANUAL without touching the gamepad.
    void _take_control_callback(const std::shared_ptr<Trigger::Request> request,
                                std::shared_ptr<Trigger::Response> response);
    /// @brief Runs once the stop has settled: re-arms the bucket if the operator has it.
    void _arm_settled();
    /// @brief Ends a handover: AUTONOMY if the drive was handed back, and the reply.
    void _finish_arm(bool armed, bool success, const std::string& message);

    static constexpr std::string FORWARD_BASE_NAME = "drive.forward";
    static constexpr std::string TURN_BASE_NAME = "drive.turn";
    static constexpr std::string LIFT_BASE_NAME = "bucket.lift";
    static constexpr std::string TILT_BASE_NAME = "bucket.tilt";
    static constexpr std::string JAWS_BASE_NAME = "bucket.jaws";
    static constexpr std::string TIMEOUT_LENGTH = "timeout_ns";

    static constexpr auto JOY_TIMEOUT = std::chrono::milliseconds(100);

    // How many zero messages to send after the drive deadman is released, or after the
    // joy stream times out, before going silent on joy_vel and bucket_actuators. twist_mux
    // gives joy_vel priority over navigation and only releases it once the topic has been
    // quiet for its timeout, so publishing zeros forever would pin the rover still under
    // nav2 as well. The bucket is on the same deadman, so it follows the same rule.
    static constexpr int STOP_BURST_LENGTH = 5;

    // How long arming waits after the zeros before calling /bucket/rearm, so
    // bucket_driver has seen them (it refuses a re-arm while a stick is held).
    static constexpr auto ARM_SETTLE_TIME = std::chrono::milliseconds(150);
    static constexpr auto REARM_TIMEOUT = std::chrono::seconds(5);
    // How old the last /joy can be and still count as the deadman's state, for arming.
    static constexpr auto JOY_STALE_TIME = std::chrono::milliseconds(500);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_subscription;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
        _twist_publisher;
    rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr
        _actuator_publisher;
    rclcpp::TimerBase::SharedPtr _joy_timeout_timer;
    int _stop_messages_remaining = 0;
    int _bucket_stop_messages_remaining = 0;
    bool _joy_timed_out = false;

    Mode _mode = Mode::MANUAL;
    /// @brief /bucket/operator_override as bucket_driver last latched it, unset if the
    /// driver has never been seen (no bucket on this rover).
    std::optional<bool> _bucket_override;

    /// @brief The /teleop/arm_autonomy call being answered, while ARMING.
    struct PendingArm
    {
        rclcpp::Service<Trigger>::SharedPtr service;
        std::shared_ptr<rmw_request_id_t> header;
    };
    std::optional<PendingArm> _pending_arm;
    std::optional<int64_t> _rearm_request_id;
    /// @brief The settle delay, then the /bucket/rearm deadline.
    rclcpp::TimerBase::SharedPtr _arm_timer;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _mode_publisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _bucket_override_subscription;
    rclcpp::Service<Trigger>::SharedPtr _arm_service;
    rclcpp::Service<Trigger>::SharedPtr _take_control_service;
    rclcpp::Client<Trigger>::SharedPtr _rearm_client;

protected:
    sensor_msgs::msg::Joy::SharedPtr _last_received_joy;
    rclcpp::Time _prev_received_joy_time;
    std::map<std::string, AxisParser> _axis_parsers;
    std::map<std::string, EnableParser> _enable_parsers;
};
