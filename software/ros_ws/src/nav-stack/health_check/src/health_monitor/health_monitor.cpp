/// @file health_monitor.cpp
/// @brief Implementation of the health monitor node.

#include "health_check/health_monitor/health_monitor.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace health_check {

HealthMonitor::HealthMonitor(const rclcpp::NodeOptions &options)
    : rclcpp::Node("health_monitor", options) {
  const auto topics = declare_parameter<std::vector<std::string>>(
      "topics", std::vector<std::string>{});
  const auto expected_rates = declare_parameter<std::vector<double>>(
      "expected_rates_hz", std::vector<double>{});
  const double window_sec =
      declare_parameter<double>("window_sec", DEFAULT_WINDOW_SEC);
  const double stale_timeout_sec =
      declare_parameter<double>("stale_timeout_sec", DEFAULT_STALE_TIMEOUT_SEC);
  const double rate_tolerance =
      declare_parameter<double>("rate_tolerance", DEFAULT_RATE_TOLERANCE);
  const double publish_rate_hz =
      declare_parameter<double>("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ);
  const double discovery_rate_hz =
      declare_parameter<double>("discovery_rate_hz", DEFAULT_DISCOVERY_RATE_HZ);

  if (topics.size() != expected_rates.size()) {
    throw std::runtime_error(
        "health_monitor: 'topics' has " + std::to_string(topics.size()) +
        " entries but "
        "'expected_rates_hz' has " +
        std::to_string(expected_rates.size()) +
        "; they are index-paired and must be the same length.");
  }
  if (window_sec <= 0.0) {
    throw std::runtime_error("health_monitor: 'window_sec' must be positive.");
  }
  if (publish_rate_hz <= 0.0 || discovery_rate_hz <= 0.0) {
    throw std::runtime_error("health_monitor: 'publish_rate_hz' and "
                             "'discovery_rate_hz' must be positive.");
  }
  if (rate_tolerance < MIN_RATE_TOLERANCE ||
      rate_tolerance > MAX_RATE_TOLERANCE) {
    throw std::runtime_error(
        "health_monitor: 'rate_tolerance' must be within [0, 1].");
  }

  _topic_monitors.reserve(topics.size());
  for (std::size_t i = 0; i < topics.size(); ++i) {
    if (expected_rates[i] <= 0.0) {
      throw std::runtime_error("health_monitor: expected rate for '" +
                               topics[i] + "' must be positive.");
    }
    _topic_monitors.push_back(
        std::make_unique<TopicMonitor>(topics[i], expected_rates[i], window_sec,
                                       stale_timeout_sec, rate_tolerance));
  }

  if (_topic_monitors.empty()) {
    RCLCPP_WARN(get_logger(),
                "No topics configured; every snapshot will be empty. Set the "
                "'topics' and 'expected_rates_hz' parameters to watch "
                "something.");
  }

  _publisher = create_publisher<interfaces::msg::SystemHealth>(
      OUTPUT_TOPIC, rclcpp::QoS(OUTPUT_QUEUE_DEPTH));

  _discovery_timer =
      create_wall_timer(std::chrono::duration<double>(1.0 / discovery_rate_hz),
                        [this] { _on_discovery_timer(); });
  _publish_timer =
      create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_hz),
                        [this] { _on_publish_timer(); });

  RCLCPP_INFO(get_logger(),
              "Health monitor up: %zu topics, publishing on %s at %.1f Hz",
              _topic_monitors.size(), OUTPUT_TOPIC.c_str(), publish_rate_hz);
}

void HealthMonitor::_on_discovery_timer() {
  for (auto &monitor : _topic_monitors) {
    monitor->try_bind(*this);
  }
}

void HealthMonitor::_on_publish_timer() {
  interfaces::msg::SystemHealth message;
  message.header.stamp = now();

  message.topics.reserve(_topic_monitors.size());
  for (auto &monitor : _topic_monitors) {
    message.topics.push_back(monitor->report(*this));
  }
  message.overall_status = _overall_status(message);

  _publisher->publish(message);
}

std::uint8_t
HealthMonitor::_overall_status(const interfaces::msg::SystemHealth &message) {
  using interfaces::msg::SystemHealth;
  using interfaces::msg::TopicHealth;

  std::uint8_t worst = SystemHealth::STATUS_OK;

  for (const auto &topic : message.topics) {
    switch (topic.status) {
    case TopicHealth::STATUS_STALE:
    case TopicHealth::STATUS_NO_PUBLISHER:
      return SystemHealth::STATUS_ERROR;
    case TopicHealth::STATUS_SLOW:
      worst = std::max(
          worst, static_cast<std::uint8_t>(SystemHealth::STATUS_DEGRADED));
      break;
    default:
      break;
    }
  }

  return worst;
}

} // namespace health_check
