/// @file topic_monitor.cpp
/// @brief Implementation of the per-topic rate and bandwidth measurement.

#include "health_check/health_monitor/topic_monitor.hpp"

#include <utility>

namespace health_check {

TopicMonitor::TopicMonitor(std::string name, double expected_rate_hz,
                           double window_sec, double stale_timeout_sec,
                           double rate_tolerance)
    : _name(std::move(name)), _expected_rate_hz(expected_rate_hz),
      _window_sec(window_sec), _stale_timeout_sec(stale_timeout_sec),
      _rate_tolerance(rate_tolerance) {}

bool TopicMonitor::try_bind(rclcpp::Node &node) {
  if (_subscription) {
    return true;
  }

  const auto graph = node.get_topic_names_and_types();
  const auto entry = graph.find(_name);
  if (entry == graph.end() || entry->second.empty()) {
    return false;
  }
  if (entry->second.size() > 1) {
    RCLCPP_WARN_ONCE(
        node.get_logger(),
        "Topic %s is advertised with %zu types; not monitoring it, because "
        "measuring only one of them would under-report the real traffic.",
        _name.c_str(), entry->second.size());
    return false;
  }

  /*
    Best-effort, volatile, keep-last. A best-effort subscription matches both
    best-effort and reliable publishers, whereas a reliable one would silently
    receive nothing from the best-effort sensor topics -- which is most of what
    this node watches. The cost is that a transient-local publisher's retained
    message is not replayed to us, which does not matter when the thing being
    measured is the live rate.
  */
  const auto qos = rclcpp::SensorDataQoS();

  _subscription = node.create_generic_subscription(
      _name, entry->second.front(), qos,
      [this, &node](std::shared_ptr<const rclcpp::SerializedMessage> message) {
        _on_message(message->size(), node.now());
      });

  RCLCPP_INFO(node.get_logger(), "Monitoring %s (%s) at an expected %.1f Hz",
              _name.c_str(), entry->second.front().c_str(), _expected_rate_hz);
  return true;
}

void TopicMonitor::_on_message(std::size_t bytes, const rclcpp::Time &now) {
  _samples.push_back({now, bytes});
  _last_message_time = now;
  ++_total_count;
}

void TopicMonitor::_prune_to(const rclcpp::Time &cutoff) {
  while (!_samples.empty() && _samples.front().stamp < cutoff) {
    _samples.pop_front();
  }
}

interfaces::msg::TopicHealth TopicMonitor::report(rclcpp::Node &node) {
  using interfaces::msg::TopicHealth;

  const rclcpp::Time now = node.now();
  _prune_to(now - rclcpp::Duration::from_seconds(_window_sec));

  TopicHealth health;
  health.name = _name;
  health.expected_rate_hz = _expected_rate_hz;
  health.publisher_count =
      static_cast<std::uint32_t>(node.count_publishers(_name));
  health.window_message_count = static_cast<std::uint32_t>(_samples.size());
  health.total_message_count = _total_count;

  std::size_t window_bytes = 0;
  for (const auto &sample : _samples) {
    window_bytes += sample.bytes;
  }
  health.measured_rate_hz = static_cast<double>(_samples.size()) / _window_sec;
  health.bandwidth_bytes_per_sec =
      static_cast<double>(window_bytes) / _window_sec;

  /*
    -1.0 rather than 0.0 for "never seen anything". A topic that has just this
    instant published and one that has never published in the node's lifetime
    would otherwise both report an age near zero, which inverts the meaning of
    the field.
  */
  health.age_sec =
      _total_count == 0 ? -1.0 : (now - _last_message_time).seconds();

  if (health.publisher_count == 0) {
    health.status = TopicHealth::STATUS_NO_PUBLISHER;
  } else if (_total_count == 0 || health.age_sec > _stale_timeout_sec) {
    health.status = TopicHealth::STATUS_STALE;
  } else if (health.measured_rate_hz <
             _expected_rate_hz * (1.0 - _rate_tolerance)) {
    health.status = TopicHealth::STATUS_SLOW;
  } else {
    health.status = TopicHealth::STATUS_OK;
  }

  return health;
}

} // namespace health_check
