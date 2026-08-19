/// @file topic_monitor.hpp
/// @brief Type-agnostic rate and bandwidth measurement for a single topic.

#ifndef HEALTH_CHECK__HEALTH_MONITOR__TOPIC_MONITOR_HPP_
#define HEALTH_CHECK__HEALTH_MONITOR__TOPIC_MONITOR_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

#include <interfaces/msg/topic_health.hpp>
#include <rclcpp/rclcpp.hpp>

namespace health_check {

/// @brief Watches one topic and reports how fast and how heavily it is
/// publishing.
///
/// The subscription is created generically, from the type name discovered on
/// the ROS graph, so the monitor never links against the message type. That is
/// what lets the watch list live in a config file: adding a topic costs a line
/// of YAML rather than a new include and a rebuild.
///
/// Binding is deferred because a topic cannot be subscribed to generically
/// until something advertises it and the type is therefore known. Until then
/// the monitor reports NO_PUBLISHER and retries on every discovery tick.
class TopicMonitor {
public:
  /// @brief Creates a monitor for one topic. Does not subscribe; call tryBind()
  /// for that.
  /// @param name Fully qualified topic name to watch.
  /// @param expected_rate_hz Rate the topic is configured to publish at, used
  /// to decide whether a measured rate counts as slow. Must be positive.
  /// @param window_sec Length of the trailing window that rate and bandwidth
  /// are averaged over.
  /// @param stale_timeout_sec Silence after which a bound topic is reported
  /// STALE.
  /// @param rate_tolerance Fraction of expected_rate_hz the measurement may
  /// fall below before the topic counts as slow, e.g. 0.5 allows half the
  /// expected rate.
  TopicMonitor(std::string name, double expected_rate_hz, double window_sec,
               double stale_timeout_sec, double rate_tolerance);

  /// @brief Attempts to create the generic subscription if it does not exist
  /// yet.
  ///
  /// Looks the topic up on the graph and binds only once exactly one type is
  /// being advertised. A topic advertised with more than one type is left
  /// unbound, because picking one arbitrarily would silently measure only part
  /// of the traffic.
  ///
  /// @param node Node the subscription is created on.
  /// @return True if the monitor is bound, whether it bound just now or already
  /// was.
  bool tryBind(rclcpp::Node &node);

  /// @brief Drops samples that have fallen outside the window and fills in a
  /// report.
  /// @param node Node used to read the clock and count publishers.
  /// @return Health of this topic as of now.
  interfaces::msg::TopicHealth report(rclcpp::Node &node);

  /// @brief Topic this monitor was configured to watch.
  const std::string &name() const { return name_; }

private:
  /// @brief Records one arrival. Called from the subscription callback.
  /// @param bytes Size of the serialised payload as delivered by the transport.
  /// @param now Arrival time, taken from the node clock.
  void onMessage(std::size_t bytes, const rclcpp::Time &now);

  /// @brief Discards samples older than the window so the averages stay
  /// trailing.
  void pruneTo(const rclcpp::Time &cutoff);

  /// @brief One received message: when it landed and how big it was.
  struct Sample {
    rclcpp::Time stamp;
    std::size_t bytes;
  };

  std::string name_;
  double expected_rate_hz_;
  double window_sec_;
  double stale_timeout_sec_;
  double rate_tolerance_;

  rclcpp::GenericSubscription::SharedPtr subscription_;
  std::deque<Sample> samples_;
  /// @brief Zero-initialised sentinel; only meaningful once total_count_ is
  /// non-zero.
  rclcpp::Time last_message_time_{0, 0, RCL_ROS_TIME};
  std::uint64_t total_count_{0};
};

} // namespace health_check

#endif // HEALTH_CHECK__HEALTH_MONITOR__TOPIC_MONITOR_HPP_
