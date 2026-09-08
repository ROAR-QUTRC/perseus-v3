#pragma once

/// @file topic_monitor.hpp
/// @brief Type-agnostic rate and bandwidth measurement for a single topic.

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
  /// @brief Creates a monitor for one topic. Does not subscribe; call
  /// try_bind() for that.
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

  /// @brief Topic this monitor was configured to watch.
  const std::string &get_name() const;

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
  bool try_bind(rclcpp::Node &node);

  /// @brief Drops samples that have fallen outside the window and fills in a
  /// report.
  /// @param node Node used to read the clock and count publishers.
  /// @return Health of this topic as of now.
  interfaces::msg::TopicHealth report(rclcpp::Node &node);

private:
  /// @brief One received message: when it landed and how big it was.
  struct sample_t {
    rclcpp::Time stamp;
    std::size_t bytes;
  };

  /// @brief Records one arrival. Called from the subscription callback.
  /// @param bytes Size of the serialised payload as delivered by the transport.
  /// @param now Arrival time, taken from the node clock.
  void _on_message(std::size_t bytes, const rclcpp::Time &now);

  /// @brief Discards samples older than the window so the averages stay
  /// trailing.
  void _prune_to(const rclcpp::Time &cutoff);

  std::string _name;
  double _expected_rate_hz;
  double _window_sec;
  double _stale_timeout_sec;
  double _rate_tolerance;

  rclcpp::GenericSubscription::SharedPtr _subscription;
  std::deque<sample_t> _samples;
  /// @brief Zero-initialised sentinel; only meaningful once _total_count is
  /// non-zero.
  rclcpp::Time _last_message_time{0, 0, RCL_ROS_TIME};
  std::uint64_t _total_count{0};
};

inline const std::string &TopicMonitor::get_name() const { return _name; }

} // namespace health_check
