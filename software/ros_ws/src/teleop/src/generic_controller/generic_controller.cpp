#include "generic_controller/generic_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>

GenericController::GenericController(const rclcpp::NodeOptions& options)
    : Node("generic_controller", options)
{
    _axis_parsers.emplace(std::make_pair(
        FORWARD_BASE_NAME, AxisParser(*this, FORWARD_BASE_NAME, true)));
    _axis_parsers.emplace(
        std::make_pair(TURN_BASE_NAME, AxisParser(*this, TURN_BASE_NAME, true)));
    _axis_parsers.emplace(
        std::make_pair(LIFT_BASE_NAME, AxisParser(*this, LIFT_BASE_NAME)));
    _axis_parsers.emplace(
        std::make_pair(TILT_BASE_NAME, AxisParser(*this, TILT_BASE_NAME)));
    _axis_parsers.emplace(
        std::make_pair(JAWS_BASE_NAME, AxisParser(*this, JAWS_BASE_NAME)));
    _joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&GenericController::_joy_callback, this,
                  std::placeholders::_1));
    _twist_publisher =
        this->create_publisher<geometry_msgs::msg::TwistStamped>("joy_vel", 10);
    _actuator_publisher = this->create_publisher<actuator_msgs::msg::Actuators>(
        "bucket_actuators", 10);
    if (this->declare_parameter("timeout_enable", "true") == "true")
    {
        _joy_timeout_timer = this->create_wall_timer(
            JOY_TIMEOUT,
            std::bind(&GenericController::_joy_timeout_callback, this));
    }
    _prev_received_joy_time = this->now();

    // Latched, like /bucket/operator_override, so a console started later sees it.
    _mode_publisher = this->create_publisher<std_msgs::msg::Bool>(
        "/teleop/autonomy_armed", rclcpp::QoS(1).reliable().transient_local());
    // A twist_mux lock (perseus/config/twist_mux.yaml): true masks navigation, so
    // nothing but the operator drives unless autonomy is armed.
    _navigation_lock_publisher =
        this->create_publisher<std_msgs::msg::Bool>("/teleop/navigation_lock", 10);
    _set_mode(Mode::MANUAL);
    _bucket_override_subscription = this->create_subscription<std_msgs::msg::Bool>(
        "/bucket/operator_override", rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::Bool& msg)
        { _bucket_override = msg.data; });
    _rearm_client = this->create_client<Trigger>("/bucket/rearm");
    _bucket_take_client = this->create_client<Trigger>("/bucket/take_control");
    // Not behind timeout_enable: this is what keeps autonomy off while in MANUAL.
    _safety_timer = this->create_wall_timer(
        SAFETY_INTERVAL, std::bind(&GenericController::_safety_callback, this));
    _arm_service = this->create_service<Trigger>(
        "/teleop/arm_autonomy",
        std::bind(&GenericController::_arm_callback, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3));
    _take_control_service = this->create_service<Trigger>(
        "/teleop/take_control",
        std::bind(&GenericController::_take_control_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Generic controller initialized");
}

void GenericController::_set_mode(Mode mode)
{
    _mode = mode;
    std_msgs::msg::Bool msg;
    msg.data = mode == Mode::AUTONOMY;
    _mode_publisher->publish(msg);
    msg.data = !msg.data;
    _navigation_lock_publisher->publish(msg);
    // At once, rather than on the next safety tick: the deadman takes the bucket from
    // autonomy the moment it is pressed, stick or no stick.
    if (mode == Mode::MANUAL)
    {
        _bucket_take_requested.reset();
        _take_bucket();
    }
}

void GenericController::_safety_callback()
{
    std_msgs::msg::Bool lock;
    lock.data = _mode != Mode::AUTONOMY;
    _navigation_lock_publisher->publish(lock);
    _take_bucket();
}

void GenericController::_take_bucket()
{
    // Only in MANUAL: while ARMING the bucket is being handed back on purpose. Unset
    // means no bucket_driver has been seen, so there is no bucket to take.
    if (_mode != Mode::MANUAL || !_bucket_override.has_value() || *_bucket_override)
    {
        _bucket_take_requested.reset();
        return;
    }
    if (_bucket_take_requested &&
        (this->now() - *_bucket_take_requested) < rclcpp::Duration(BUCKET_TAKE_RETRY))
        return;
    // Unset only in the constructor's first _set_mode, before the client exists.
    if (!_bucket_take_client)
        return;
    if (!_bucket_take_client->service_is_ready())
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "Manual, but autonomy still has the bucket: bucket_driver "
                              "does not serve /bucket/take_control - rebuild payloads on "
                              "the rover and restart bucket_driver");
        return;
    }
    RCLCPP_WARN(this->get_logger(), "Manual: taking the bucket from autonomy");
    _bucket_take_requested = this->now();
    _bucket_take_client->async_send_request(std::make_shared<Trigger::Request>());
}

void GenericController::_publish_stop_bursts()
{
    if (_stop_messages_remaining > 0)
    {
        --_stop_messages_remaining;
        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.stamp = this->now();
        _twist_publisher->publish(twist_msg);
    }
    if (_bucket_stop_messages_remaining > 0)
    {
        --_bucket_stop_messages_remaining;
        actuator_msgs::msg::Actuators actuator_msg;
        actuator_msg.velocity = {0.0, 0.0, 0.0};
        actuator_msg.header.stamp = this->now();
        _actuator_publisher->publish(actuator_msg);
    }
}

void GenericController::_arm_callback(rclcpp::Service<Trigger>::SharedPtr service,
                                      std::shared_ptr<rmw_request_id_t> header,
                                      std::shared_ptr<Trigger::Request> /*request*/)
{
    const auto reply = [&service, &header](bool success, const std::string& message)
    {
        Trigger::Response response;
        response.success = success;
        response.message = message;
        service->send_response(*header, response);
    };

    if (_mode == Mode::AUTONOMY)
    {
        reply(true, "already armed");
        return;
    }
    if (_mode == Mode::ARMING)
    {
        reply(false, "already arming");
        return;
    }
    // A stale message doesn't count: a gamepad that dropped out with the deadman
    // down would otherwise block arming until it came back.
    const bool joy_live =
        _last_received_joy && (this->now() - _prev_received_joy_time) < rclcpp::Duration(JOY_STALE_TIME);
    if (joy_live && _enable_parsers.at(FORWARD_BASE_NAME + ".enable").get_value())
    {
        reply(false, "release the deadman first");
        return;
    }

    _pending_arm = PendingArm{service, header};
    _set_mode(Mode::ARMING);
    RCLCPP_INFO(this->get_logger(), "Arming autonomy: stopping");
    // Whatever was mid-burst finishes now, so the drive stops and bucket_driver's
    // latest speeds are zero before the re-arm, which it refuses otherwise.
    _stop_messages_remaining = std::max(_stop_messages_remaining, 1);
    _bucket_stop_messages_remaining = std::max(_bucket_stop_messages_remaining, 1);
    while (_stop_messages_remaining > 0 || _bucket_stop_messages_remaining > 0)
        _publish_stop_bursts();

    _arm_timer = this->create_wall_timer(ARM_SETTLE_TIME,
                                         [this]
                                         {
                                             _arm_timer->cancel();
                                             _arm_settled();
                                         });
}

void GenericController::_arm_settled()
{
    if (_mode != Mode::ARMING)
        return;

    if (!_bucket_override.value_or(false))
    {
        _finish_arm(true, true,
                    std::string("drive: back to navigation; bucket: ") +
                        (_bucket_override ? "autonomy already has it"
                                          : "no bucket_driver seen"));
        return;
    }
    if (!_rearm_client->service_is_ready())
    {
        _finish_arm(true, false,
                    "drive: back to navigation; bucket: the operator still has it - "
                    "/bucket/rearm is not available");
        return;
    }

    auto future = _rearm_client->async_send_request(
        std::make_shared<Trigger::Request>(),
        [this](rclcpp::Client<Trigger>::SharedFuture answered)
        {
            _rearm_request_id.reset();
            if (_mode != Mode::ARMING)
                return;
            const auto response = answered.get();
            _finish_arm(true, response->success,
                        "drive: back to navigation; bucket: " +
                            (response->success ? response->message
                                               : "the operator still has it - " +
                                                     response->message));
        });
    _rearm_request_id = future.request_id;
    _arm_timer = this->create_wall_timer(
        REARM_TIMEOUT,
        [this]
        {
            if (_mode == Mode::ARMING)
                _finish_arm(true, false,
                            "drive: back to navigation; bucket: the operator still has "
                            "it - no answer from /bucket/rearm");
        });
}

void GenericController::_finish_arm(bool armed, bool success, const std::string& message)
{
    if (_arm_timer)
        _arm_timer->cancel();
    if (_rearm_request_id)
        _rearm_client->remove_pending_request(*_rearm_request_id);
    _rearm_request_id.reset();

    // Armed even when the bucket refused: the drive is handed back either way, and
    // the deadman still takes everything back.
    _set_mode(armed ? Mode::AUTONOMY : Mode::MANUAL);
    if (armed)
        RCLCPP_INFO(this->get_logger(), "Autonomy armed - %s", message.c_str());
    if (_pending_arm)
    {
        Trigger::Response response;
        response.success = success;
        response.message = message;
        _pending_arm->service->send_response(*_pending_arm->header, response);
        _pending_arm.reset();
    }
}

void GenericController::_take_control_callback(
    const std::shared_ptr<Trigger::Request> /*request*/,
    std::shared_ptr<Trigger::Response> response)
{
    if (_mode == Mode::ARMING)
        _finish_arm(false, false, "aborted: the operator took control");
    if (_mode == Mode::AUTONOMY)
    {
        RCLCPP_WARN(this->get_logger(), "Operator took control (console)");
        // Navigation's last command is masked from here on, so stop the rover
        // explicitly rather than leaving it to the drive controller's timeout.
        _stop_messages_remaining = STOP_BURST_LENGTH;
        _bucket_stop_messages_remaining = STOP_BURST_LENGTH;
        _publish_stop_bursts();
    }
    _set_mode(Mode::MANUAL);
    response->success = true;
    response->message = "manual: autonomy is locked out; the gamepad drives while the "
                        "deadman is held";
}

void GenericController::_joy_timeout_callback(void)
{
    static const auto timeout_length = rcl_time_point_value_t(
        this->declare_parameter(TIMEOUT_LENGTH, 500000000));
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Timeout length is %ld ns", timeout_length);
    if ((this->now().nanoseconds() - _prev_received_joy_time.nanoseconds()) >
        timeout_length)
    {
        // Armed, nothing goes out. Otherwise a short burst of zeros stops the rover and
        // the bucket, then both topics go quiet; navigation stays locked out until
        // autonomy is armed, and bucket_driver times out to zero on its own.
        if (_mode != Mode::MANUAL)
            return;
        _publish_stop_bursts();

        // Warn user if controller is disconnected, once per dropout
        if (!_joy_timed_out)
            RCLCPP_WARN(this->get_logger(), "Joy timeout, stopped the drive and the bucket");
        _joy_timed_out = true;
    }
}

void GenericController::_joy_callback(
    const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // How long the dropout was, so a flaky link or a too-tight timeout_ns shows up
    if (_joy_timed_out)
        RCLCPP_WARN(this->get_logger(), "Joy back after %ld ms",
                    static_cast<long>(
                        (this->now() - _prev_received_joy_time).nanoseconds() / 1000000));
    _last_received_joy = msg;
    _prev_received_joy_time = this->now();
    _joy_timed_out = false;
    double forward = _axis_parsers.at(FORWARD_BASE_NAME).get_value();
    double turn = _axis_parsers.at(TURN_BASE_NAME).get_value();
    double lift = _axis_parsers.at(LIFT_BASE_NAME).get_value();
    double tilt = _axis_parsers.at(TILT_BASE_NAME).get_value();
    double jaws = _axis_parsers.at(JAWS_BASE_NAME).get_value();

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Forward: %+2.2f, Turn: %+2.2f, Lift: %+2.2f, Tilt: "
                          "%+2.2f, Jaws: %+2.2f",
                          forward, turn, lift, tilt, jaws);

    // joy_vel is only published while the drive deadman is held, plus a short burst of
    // zeros after it is released to stop the rover. Publishing zeros whenever the stick
    // is idle would keep twist_mux's highest-priority input alive and block web_ui and
    // the keyboard; navigation is held off by /teleop/navigation_lock instead. With the
    // deadman released forward and turn are already zero, so the burst carries exactly
    // what used to be sent. The bucket is on the same deadman.
    bool drive_enabled =
        _enable_parsers.at(FORWARD_BASE_NAME + ".enable").get_value();

    if (drive_enabled && _mode == Mode::ARMING)
        _finish_arm(false, false, "aborted: the operator took control");
    else if (drive_enabled && _mode == Mode::AUTONOMY)
    {
        RCLCPP_WARN(this->get_logger(), "Operator took control (deadman)");
        _set_mode(Mode::MANUAL);
    }
    if (_mode != Mode::MANUAL)
        return;

    if (drive_enabled)
    {
        _stop_messages_remaining = STOP_BURST_LENGTH;
        _bucket_stop_messages_remaining = STOP_BURST_LENGTH;

        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.twist.linear.x = forward;
        twist_msg.twist.angular.z = turn;

        twist_msg.header.stamp = this->now();
        // Publish twist message
        _twist_publisher->publish(twist_msg);

        actuator_msgs::msg::Actuators actuator_msg;
        actuator_msg.velocity.push_back(lift);
        actuator_msg.velocity.push_back(tilt);
        actuator_msg.velocity.push_back(jaws);

        actuator_msg.header.stamp = this->now();
        // Publish actuator message
        _actuator_publisher->publish(actuator_msg);
    }
    else
        _publish_stop_bursts();
}

GenericController::AxisParser::AxisParser(GenericController& parent,
                                          const std::string& param_base_name,
                                          bool has_enable)
    : _parent(parent),
      _param_base_name(param_base_name),
      _has_enable(has_enable)
{
    using namespace rcl_interfaces::msg;

    ParameterDescriptor axis_descriptor{};
    axis_descriptor.type = ParameterType::PARAMETER_INTEGER;
    axis_descriptor.description = "The analog axis to use for parsing";

    ParameterDescriptor button_positive_descriptor{};
    button_positive_descriptor.type = ParameterType::PARAMETER_INTEGER;
    button_positive_descriptor.description =
        "The button to use which will set the axis to its maximum";

    ParameterDescriptor button_negative_descriptor{};
    button_negative_descriptor.type = ParameterType::PARAMETER_INTEGER;
    button_negative_descriptor.description =
        "The button to use which will set the axis to its minimum";

    _int_param_map = {
        {"axis", {-1, axis_descriptor}},
        {"button_positive", {-1, button_positive_descriptor}},
        {"button_negative", {-1, button_negative_descriptor}},
    };

    ParameterDescriptor scaling_descriptor{};
    scaling_descriptor.type = ParameterType::PARAMETER_DOUBLE;
    scaling_descriptor.description = "The axis output scale";

    ParameterDescriptor deadband_descriptor{};
    deadband_descriptor.type = ParameterType::PARAMETER_DOUBLE;
    deadband_descriptor.description =
        "The deadband range for the axis. If positive is NaN, no deadband is "
        "applied, and if negative is NaN, it default to negative positive "
        "deadband";

    ParameterDescriptor turbo_descriptor{};
    turbo_descriptor.type = ParameterType::PARAMETER_DOUBLE;
    turbo_descriptor.description = "Turbo scaling value";

    _double_param_map = {
        {"scaling",
         {std::numeric_limits<double>::quiet_NaN(), scaling_descriptor}},
        {"turbo", {std::numeric_limits<double>::quiet_NaN(), turbo_descriptor}},
        {"deadband_positive", {0.08, deadband_descriptor}},
        {"deadband_negative",
         {std::numeric_limits<double>::quiet_NaN(), deadband_descriptor}}};

    ParameterDescriptor follows_descriptor{};
    follows_descriptor.type = ParameterType::PARAMETER_STRING;
    follows_descriptor.description =
        "The enable configuration to base initial parameters off of";
    _string_param_map = {{"follows", {"", follows_descriptor}}};

    _parent.declare_parameters(param_base_name, _int_param_map);
    _parent.declare_parameters(param_base_name, _double_param_map);
    _parent.declare_parameters(param_base_name, _string_param_map);

    ParameterDescriptor hold_descriptor{};
    hold_descriptor.type = ParameterType::PARAMETER_BOOL;
    hold_descriptor.description =
        "Whether or not to hold the previous value if no input received";
    _parent.declare_parameter(param_base_name + ".hold", false, hold_descriptor);

    if (has_enable)
    {
        std::string enable_name = param_base_name + ".enable";
        std::string turbo_name = param_base_name + ".turbo_enable";
        _parent._enable_parsers.emplace(
            std::make_pair(enable_name, EnableParser(_parent, enable_name)));
        _parent._enable_parsers.emplace(
            std::make_pair(turbo_name, EnableParser(_parent, turbo_name)));
    }
}

double GenericController::AxisParser::get_value()
{
    // Only .enable is a deadman switch not .turbo_enable
    if (_has_enable &&
        !_parent._enable_parsers.at(_param_base_name + ".enable").get_value())
        return 0.0;

    auto params = _resolve_params();

    double axis_value = 0.0;
    int axis_idx = params[AXIS_IDX].as_int();
    bool has_analog_axis = axis_idx >= 0;
    bool hold = _parent.get_parameter(_param_base_name + ".hold").as_bool();
    if (has_analog_axis)
        axis_value = _parent._last_received_joy->axes[axis_idx];

    double scaling = params[SCALING_IDX].as_double();
    if (_has_enable &&
        _parent._enable_parsers.at(_param_base_name + ".turbo_enable")
            .get_value())
    {
        scaling = params[TURBO_IDX].as_double();
        if (!std::isfinite(scaling))
            scaling = 2.0;
    }
    if (!std::isfinite(scaling))
        scaling = 1.0;

    double deadband_positive = params[DEADBAND_POSITIVE_IDX].as_double();
    double deadband_negative = params[DEADBAND_NEGATIVE_IDX].as_double();
    if (!std::isfinite(deadband_negative))
        deadband_negative = -deadband_positive;

    if (std::isfinite(deadband_positive))
    {
        bool clamped =
            axis_value < deadband_positive && axis_value > deadband_negative;
        if (clamped)
            axis_value = 0.0;
        else if (axis_value > deadband_positive)
            axis_value -= deadband_positive;
        else if (axis_value < deadband_negative)
            axis_value -= deadband_negative;

        // re-normalize the axis value
        if (!clamped)
        {
            if (axis_value > 0.0)
                axis_value /= (1.0 - std::abs(deadband_positive));
            else
                axis_value /= (1.0 - std::abs(deadband_negative));
        }
    }

    bool button_positive = false;
    if (int button = params[BUTTON_POSITIVE_IDX].as_int(); button >= 0)
        button_positive = _parent._last_received_joy->buttons[button];

    bool button_negative = false;
    if (int button = params[BUTTON_NEGATIVE_IDX].as_int(); button >= 0)
        button_negative = _parent._last_received_joy->buttons[button];

    bool both_buttons = button_positive && button_negative;
    if (!both_buttons)
    {
        if (button_positive)
            axis_value = 1.0;
        if (button_negative)
            axis_value = -1.0;
    }
    else
        axis_value = 0.0;

    axis_value *= scaling;
    bool should_update_analog_axis =
        has_analog_axis && (!hold || (abs(axis_value) > 0));
    if (button_positive || button_negative || should_update_analog_axis)
        _last_value = axis_value;
    else if (hold)
        return _last_value;

    return axis_value;
}

std::vector<rclcpp::Parameter>
GenericController::AxisParser::_resolve_params()
{
    // TODO: Memoize this with invalidation on parameter changes
    auto params = _parent.get_parameters(_get_param_names(_param_base_name));
    // has an axis parser with that key
    if (auto followed =
            _parent._axis_parsers.find(params[FOLLOWS_IDX].as_string());
        followed != _parent._axis_parsers.end())
    {
        auto followed_params = followed->second._resolve_params();
        if (auto axis = followed_params[AXIS_IDX]; axis.as_int() >= 0)
            params[AXIS_IDX] = axis;
        if (auto button_positive = followed_params[BUTTON_POSITIVE_IDX];
            button_positive.as_int() >= 0)
            params[BUTTON_POSITIVE_IDX] = button_positive;
        if (auto button_negative = followed_params[BUTTON_NEGATIVE_IDX];
            button_negative.as_int() >= 0)
            params[BUTTON_NEGATIVE_IDX] = button_negative;
        if (auto scaling = followed_params[SCALING_IDX];
            std::isfinite(scaling.as_double()))
            params[SCALING_IDX] = scaling;
        if (auto deadband_positive = followed_params[DEADBAND_POSITIVE_IDX];
            std::isfinite(deadband_positive.as_double()))
            params[DEADBAND_POSITIVE_IDX] = deadband_positive;
        if (auto deadband_negative = followed_params[DEADBAND_NEGATIVE_IDX];
            std::isfinite(deadband_negative.as_double()))
            params[DEADBAND_NEGATIVE_IDX] = deadband_negative;
        if (auto turbo = followed_params[TURBO_IDX];
            std::isfinite(turbo.as_double()))
            params[TURBO_IDX] = turbo;
    }

    return params;
}

std::vector<std::string>
GenericController::AxisParser::_get_param_names(const std::string& base_name)
{
    return {base_name + ".axis",
            base_name + ".button_positive",
            base_name + ".button_negative",
            base_name + ".scaling",
            base_name + ".follows",
            base_name + ".deadband_positive",
            base_name + ".deadband_negative",
            base_name + ".turbo"};
}

bool GenericController::AxisParser::_parent_has_all_params(
    const std::string& follows_name)
{
    for (const auto& param : _get_param_names(follows_name))
        if (!_parent.has_parameter(param))
            return false;

    return true;
}

bool GenericController::AxisParser::_parent_has_any_params(
    const std::string& follows_name)
{
    for (const auto& param : _get_param_names(follows_name))
    {
        if (_parent.has_parameter(param))
            return true;
    }

    return false;
}

GenericController::EnableParser::EnableParser(
    GenericController& parent, const std::string& param_base_name,
    bool is_turbo)
    : _parent(parent),
      _param_base_name(param_base_name)
{
    (void)is_turbo;
    using namespace rcl_interfaces::msg;

    auto [vals, was_inserted] = _parent._axis_parsers.emplace(
        std::make_pair(param_base_name, AxisParser(parent, param_base_name)));
    if (!was_inserted)
    {
        throw std::runtime_error("Failed to insert axis parser");
    }

    ParameterDescriptor threshold_descriptor{};
    threshold_descriptor.type = ParameterType::PARAMETER_DOUBLE;
    threshold_descriptor.description = "The threshold to compare against";
    _parent.declare_parameter(param_base_name + ".threshold",
                              std::numeric_limits<double>::quiet_NaN(),
                              threshold_descriptor);

    ParameterDescriptor is_less_than_descriptor{};
    is_less_than_descriptor.type = ParameterType::PARAMETER_BOOL;
    is_less_than_descriptor.description =
        "Whether or not to enable when the axis value is less than the threshold";
    _parent.declare_parameter(param_base_name + ".is_less_than", false,
                              threshold_descriptor);

    // if (is_turbo)
    // {
    //     ParameterDescriptor turbo_descriptor{};
    //     turbo_descriptor.type = ParameterType::PARAMETER_DOUBLE;
    //     turbo_descriptor.description = "";
    //     _parent.declare_parameter(param_base_name + ".turbo_speed", 2.0,
    //     turbo_descriptor);
    // }
}

std::vector<rclcpp::Parameter>
GenericController::EnableParser::_resolve_params()
{
    // TODO: Memoize this with invalidation on parameter changes
    auto params = _parent.get_parameters(_get_param_names(_param_base_name));
    // has an axis parser with that key
    if (auto followed =
            _parent._enable_parsers.find(params[FOLLOWS_IDX].as_string());
        followed != _parent._enable_parsers.end())
    {
        auto followed_params = followed->second._resolve_params();
        if (auto threshold = followed_params[THRESHOLD_IDX];
            std::isfinite(threshold.as_double()))
        {
            params[THRESHOLD_IDX] = threshold;
            params[IS_LT_IDX] = followed_params[IS_LT_IDX];
        }
    }

    return params;
}

std::vector<std::string> GenericController::EnableParser::_get_param_names(
    const std::string& base_name)
{
    return {base_name + ".follows", base_name + ".threshold",
            base_name + ".is_less_than"};
}

bool GenericController::EnableParser::get_value()
{
    auto params = _resolve_params();

    double threshold = params[THRESHOLD_IDX].as_double();
    if (!std::isfinite(threshold))
        threshold = 0.5;

    bool is_less_than = params[IS_LT_IDX].as_bool();
    double axis_value = _parent._axis_parsers.at(_param_base_name).get_value();
    if (is_less_than)
        return axis_value < threshold;
    return axis_value > threshold;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options{};
    try
    {
        node_options.allow_undeclared_parameters(true);
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting generic controller node");
        auto node = std::make_shared<GenericController>(node_options);
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"),
                     "Error running generic controller: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
