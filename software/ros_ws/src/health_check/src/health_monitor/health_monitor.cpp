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
  const double window_sec = declare_parameter<double>("window_sec", 5.0);
  const double stale_timeout_sec =
      declare_parameter<double>("stale_timeout_sec", 2.0);
  const double rate_tolerance =
      declare_parameter<double>("rate_tolerance", 0.5);
  const double publish_rate_hz =
      declare_parameter<double>("publish_rate_hz", 1.0);
  const double discovery_rate_hz =
      declare_parameter<double>("discovery_rate_hz", 0.5);
  const auto interface_name =
      declare_parameter<std::string>("interface_name", "wlan0");
  const auto ping_host = declare_parameter<std::string>("ping_host", "");
  const double ping_period_sec =
      declare_parameter<double>("ping_period_sec", 5.0);
  const double ping_timeout_sec =
      declare_parameter<double>("ping_timeout_sec", 1.0);

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
  if (rate_tolerance < 0.0 || rate_tolerance > 1.0) {
    throw std::runtime_error(
        "health_monitor: 'rate_tolerance' must be within [0, 1].");
  }

  topic_monitors_.reserve(topics.size());
  for (std::size_t i = 0; i < topics.size(); ++i) {
    if (expected_rates[i] <= 0.0) {
      throw std::runtime_error("health_monitor: expected rate for '" +
                               topics[i] + "' must be positive.");
    }
    topic_monitors_.push_back(
        std::make_unique<TopicMonitor>(topics[i], expected_rates[i], window_sec,
                                       stale_timeout_sec, rate_tolerance));
  }

  if (topic_monitors_.empty()) {
    RCLCPP_WARN(get_logger(),
                "No topics configured; publishing link health only. Set the "
                "'topics' and "
                "'expected_rates_hz' parameters to watch something.");
  }

  link_monitor_ = std::make_unique<LinkMonitor>(
      interface_name, ping_host, ping_period_sec, ping_timeout_sec);
  link_monitor_->start(get_logger());

  publisher_ = create_publisher<interfaces::msg::SystemHealth>("health",
                                                               rclcpp::QoS(10));

  discovery_timer_ =
      create_wall_timer(std::chrono::duration<double>(1.0 / discovery_rate_hz),
                        [this] { onDiscoveryTimer(); });
  publish_timer_ =
      create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_hz),
                        [this] { onPublishTimer(); });

  RCLCPP_INFO(
      get_logger(),
      "Health monitor up: %zu topics, interface %s, publishing on 'health' at "
      "%.1f Hz",
      topic_monitors_.size(), interface_name.c_str(), publish_rate_hz);
}

void HealthMonitor::onDiscoveryTimer() {
  for (auto &monitor : topic_monitors_) {
    monitor->tryBind(*this);
  }
}

void HealthMonitor::onPublishTimer() {
  interfaces::msg::SystemHealth message;
  message.header.stamp = now();

  message.topics.reserve(topic_monitors_.size());
  for (auto &monitor : topic_monitors_) {
    message.topics.push_back(monitor->report(*this));
  }
  message.link = link_monitor_->report(now());
  message.overall_status = overallStatus(message);

  publisher_->publish(message);
}

std::uint8_t
HealthMonitor::overallStatus(const interfaces::msg::SystemHealth &message) {
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

  /*
    A configured host that has stopped answering is an error: it is the same
    class of problem as a topic losing its publisher. Errors and drops on the
    interface are only a degradation, since a link can shed packets and still
    carry the stack.
  */
  if (message.link.ping_enabled && !message.link.reachable) {
    return SystemHealth::STATUS_ERROR;
  }
  const auto &link = message.link;
  if (link.rx_errors > 0 || link.tx_errors > 0 || link.rx_dropped > 0 ||
      link.tx_dropped > 0) {
    worst = std::max(worst,
                     static_cast<std::uint8_t>(SystemHealth::STATUS_DEGRADED));
  }

  return worst;
}

} // namespace health_check
