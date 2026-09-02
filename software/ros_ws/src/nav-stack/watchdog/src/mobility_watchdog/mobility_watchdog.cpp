/// @file mobility_watchdog.cpp
/// @brief Implementation of the mobility slip-detection node.

#include "watchdog/mobility_watchdog/mobility_watchdog.hpp"

#include <algorithm>
#include <functional>

namespace watchdog {
namespace {
/// @brief Queue depth used for the command, odometry, and status topics.
constexpr int QOS_DEPTH = 10;
/// @brief Efficiency reported for a channel that is not being evaluated.
///
/// Negative rather than zero, so "nothing was commanded" is distinguishable
/// from "motion was commanded and entirely lost". See MobilityStatus.msg.
constexpr double NOT_EVALUATING_EFFICIENCY = -1.0;
} // namespace

MobilityWatchdog::MobilityWatchdog(const rclcpp::NodeOptions &options)
    : rclcpp::Node("mobility_watchdog", options),
      _latest_cmd_vel_stamp(this->now()), _last_publish_stamp(this->now()) {
  std::string cmd_vel_topic;
  std::string odometry_topic;
  std::string output_topic;
  _load_parameters(cmd_vel_topic, odometry_topic, output_topic);

  _cmd_vel_subscription =
      this->create_subscription<geometry_msgs::msg::TwistStamped>(
          cmd_vel_topic, QOS_DEPTH,
          std::bind(&MobilityWatchdog::_cmd_vel_callback, this,
                    std::placeholders::_1));

  _odometry_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
      odometry_topic, QOS_DEPTH,
      std::bind(&MobilityWatchdog::_odometry_callback, this,
                std::placeholders::_1));

  _status_publisher = this->create_publisher<interfaces::msg::MobilityStatus>(
      output_topic, QOS_DEPTH);
}

void MobilityWatchdog::_load_parameters(std::string &cmd_vel_topic_out,
                                        std::string &odometry_topic_out,
                                        std::string &output_topic_out) {
  cmd_vel_topic_out =
      this->declare_parameter("cmd_vel_topic", DEFAULT_CMD_VEL_TOPIC);
  odometry_topic_out =
      this->declare_parameter("odometry_topic", DEFAULT_ODOMETRY_TOPIC);
  output_topic_out =
      this->declare_parameter("output_topic", DEFAULT_OUTPUT_TOPIC);
  _command_timeout_s =
      this->declare_parameter("command_timeout_s", DEFAULT_COMMAND_TIMEOUT_S);
  _publish_period_s =
      this->declare_parameter("publish_period_s", DEFAULT_PUBLISH_PERIOD_S);

  const double min_linear_speed_mps = this->declare_parameter(
      "min_linear_speed_mps", DEFAULT_MIN_LINEAR_SPEED_MPS);
  const double min_angular_speed_rps = this->declare_parameter(
      "min_angular_speed_rps", DEFAULT_MIN_ANGULAR_SPEED_RPS);
  const double slip_ratio_threshold = this->declare_parameter(
      "slip_ratio_threshold", DEFAULT_SLIP_RATIO_THRESHOLD);
  const double detection_window_s =
      this->declare_parameter("detection_window_s", DEFAULT_DETECTION_WINDOW_S);
  const double activation_grace_period_s = this->declare_parameter(
      "activation_grace_period_s", DEFAULT_ACTIVATION_GRACE_PERIOD_S);

  _linear_monitor = SlipChannelMonitor(SlipChannelMonitor::config_t{
      .min_commanded_magnitude = min_linear_speed_mps,
      .slip_ratio_threshold = slip_ratio_threshold,
      .detection_window_s = detection_window_s,
      .activation_grace_period_s = activation_grace_period_s,
  });
  _angular_monitor = SlipChannelMonitor(SlipChannelMonitor::config_t{
      .min_commanded_magnitude = min_angular_speed_rps,
      .slip_ratio_threshold = slip_ratio_threshold,
      .detection_window_s = detection_window_s,
      .activation_grace_period_s = activation_grace_period_s,
  });
}

void MobilityWatchdog::_cmd_vel_callback(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  _latest_cmd_vel = msg->twist;
  _latest_cmd_vel_stamp = this->now();
  _has_cmd_vel = true;
}

void MobilityWatchdog::_odometry_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  const rclcpp::Time now = this->now();
  const bool command_fresh = _is_cached_command_fresh(now);

  // A stale or absent command is evaluated as "commanding nothing" rather than
  // skipped outright, so each monitor still sees the zero and clears its window
  // instead of coasting on whatever it last measured.
  const double commanded_linear =
      command_fresh ? _latest_cmd_vel.linear.x : 0.0;
  const double commanded_angular =
      command_fresh ? _latest_cmd_vel.angular.z : 0.0;

  // The monitors are fed every odometry sample regardless of the publish
  // period below. Throttling the detection as well as the reporting would
  // shrink the sliding window to a handful of samples and change what counts as
  // slip, rather than just how often the verdict is announced.
  _linear_monitor.update(commanded_linear, msg->twist.twist.linear.x, now);
  _angular_monitor.update(commanded_angular, msg->twist.twist.angular.z, now);

  const bool is_slipping =
      _linear_monitor.is_slipping() || _angular_monitor.is_slipping();
  // A change in the verdict goes out immediately instead of waiting out the
  // period. This is a watchdog: it already costs a detection window to make up
  // its mind, and adding up to publish_period_s of reporting latency on top of
  // that would undermine the point of having one. Steady state is still one
  // message per period.
  if (is_slipping == _was_slipping && !_should_publish(now)) {
    return;
  }

  _was_slipping = is_slipping;
  _last_publish_stamp = now;
  _has_published = true;
  _publish_status(*msg);
}

bool MobilityWatchdog::_is_cached_command_fresh(const rclcpp::Time &now) const {
  return _has_cmd_vel &&
         (now - _latest_cmd_vel_stamp).seconds() <= _command_timeout_s;
}

bool MobilityWatchdog::_should_publish(const rclcpp::Time &now) const {
  // A zero or negative period disables throttling, restoring one message per
  // odometry sample for anyone who wants the raw stream back.
  return !_has_published || _publish_period_s <= 0.0 ||
         (now - _last_publish_stamp).seconds() >= _publish_period_s;
}

double
MobilityWatchdog::_efficiency_percent(const SlipChannelMonitor &monitor) {
  if (!monitor.is_evaluating()) {
    return NOT_EVALUATING_EFFICIENCY;
  }
  // Clamped at both ends: the slip ratio can exceed 1 when the measured motion
  // opposes the command, and go negative when the robot overruns it, and
  // neither reads sensibly as a percentage of the command achieved.
  return std::clamp(1.0 - monitor.slip_ratio(), 0.0, 1.0) * 100.0;
}

void MobilityWatchdog::_publish_status(
    const nav_msgs::msg::Odometry &odometry) const {
  interfaces::msg::MobilityStatus status;
  status.header = odometry.header;
  status.is_evaluating =
      _linear_monitor.is_evaluating() || _angular_monitor.is_evaluating();
  status.is_slipping =
      _linear_monitor.is_slipping() || _angular_monitor.is_slipping();
  status.linear_slip_ratio = _linear_monitor.slip_ratio();
  status.angular_slip_ratio = _angular_monitor.slip_ratio();
  status.linear_efficiency = _efficiency_percent(_linear_monitor);
  status.angular_efficiency = _efficiency_percent(_angular_monitor);
  status.commanded_linear_velocity = _linear_monitor.commanded_value();
  status.measured_linear_velocity = _linear_monitor.measured_value();
  status.commanded_angular_velocity = _angular_monitor.commanded_value();
  status.measured_angular_velocity = _angular_monitor.measured_value();

  _status_publisher->publish(status);
}

} // namespace watchdog
