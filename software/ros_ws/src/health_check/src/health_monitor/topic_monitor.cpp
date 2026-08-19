/// @file topic_monitor.cpp
/// @brief Implementation of the per-topic rate and bandwidth measurement.

#include "health_check/health_monitor/topic_monitor.hpp"

#include <utility>

namespace health_check {

TopicMonitor::TopicMonitor(std::string name, double expected_rate_hz,
                           double window_sec, double stale_timeout_sec,
                           double rate_tolerance)
    : name_(std::move(name)), expected_rate_hz_(expected_rate_hz),
      window_sec_(window_sec), stale_timeout_sec_(stale_timeout_sec),
      rate_tolerance_(rate_tolerance) {}

bool TopicMonitor::tryBind(rclcpp::Node &node) {
  if (subscription_) {
    return true;
  }

  const auto graph = node.get_topic_names_and_types();
  const auto entry = graph.find(name_);
  if (entry == graph.end() || entry->second.empty()) {
    return false;
  }
  if (entry->second.size() > 1) {
    RCLCPP_WARN_ONCE(
        node.get_logger(),
        "Topic %s is advertised with %zu types; not monitoring it, because "
        "measuring only one of them would under-report the real traffic.",
        name_.c_str(), entry->second.size());
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

  subscription_ = node.create_generic_subscription(
      name_, entry->second.front(), qos,
      [this, &node](std::shared_ptr<const rclcpp::SerializedMessage> message) {
        onMessage(message->size(), node.now());
      });

  RCLCPP_INFO(node.get_logger(), "Monitoring %s (%s) at an expected %.1f Hz",
              name_.c_str(), entry->second.front().c_str(), expected_rate_hz_);
  return true;
}

void TopicMonitor::onMessage(std::size_t bytes, const rclcpp::Time &now) {
  samples_.push_back({now, bytes});
  last_message_time_ = now;
  ++total_count_;
}

void TopicMonitor::pruneTo(const rclcpp::Time &cutoff) {
  while (!samples_.empty() && samples_.front().stamp < cutoff) {
    samples_.pop_front();
  }
}

interfaces::msg::TopicHealth TopicMonitor::report(rclcpp::Node &node) {
  using interfaces::msg::TopicHealth;

  const rclcpp::Time now = node.now();
  pruneTo(now - rclcpp::Duration::from_seconds(window_sec_));

  TopicHealth health;
  health.name = name_;
  health.expected_rate_hz = expected_rate_hz_;
  health.publisher_count =
      static_cast<std::uint32_t>(node.count_publishers(name_));
  health.window_message_count = static_cast<std::uint32_t>(samples_.size());
  health.total_message_count = total_count_;

  std::size_t window_bytes = 0;
  for (const auto &sample : samples_) {
    window_bytes += sample.bytes;
  }
  health.measured_rate_hz = static_cast<double>(samples_.size()) / window_sec_;
  health.bandwidth_bytes_per_sec =
      static_cast<double>(window_bytes) / window_sec_;

  /*
    -1.0 rather than 0.0 for "never seen anything". A topic that has just this
    instant published and one that has never published in the node's lifetime
    would otherwise both report an age near zero, which inverts the meaning of
    the field.
  */
  health.age_sec =
      total_count_ == 0 ? -1.0 : (now - last_message_time_).seconds();

  if (health.publisher_count == 0) {
    health.status = TopicHealth::STATUS_NO_PUBLISHER;
  } else if (total_count_ == 0 || health.age_sec > stale_timeout_sec_) {
    health.status = TopicHealth::STATUS_STALE;
  } else if (health.measured_rate_hz <
             expected_rate_hz_ * (1.0 - rate_tolerance_)) {
    health.status = TopicHealth::STATUS_SLOW;
  } else {
    health.status = TopicHealth::STATUS_OK;
  }

  return health;
}

} // namespace health_check
