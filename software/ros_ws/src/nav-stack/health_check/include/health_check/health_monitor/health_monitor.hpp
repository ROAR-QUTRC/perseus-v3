#pragma once

/// @file health_monitor.hpp
/// @brief Node that aggregates topic and link health and publishes it.

#include <memory>
#include <string>
#include <vector>

#include <interfaces/msg/system_health.hpp>
#include <rclcpp/rclcpp.hpp>

#include "health_check/health_monitor/link_monitor.hpp"
#include "health_check/health_monitor/topic_monitor.hpp"

namespace health_check {

/// @brief Watches the sensing and estimation stack and publishes a health
/// snapshot.
///
/// The watch list is a pair of parallel parameters, `topics` and
/// `expected_rates_hz`, rather than a list of structs, because ROS parameters
/// cannot express a list of structs directly. They are checked for equal length
/// at startup and the node refuses to run if they disagree, which turns a
/// silently mismatched config into an immediate failure.
class HealthMonitor : public rclcpp::Node {
public:
  /// @brief Builds the node, reads parameters and starts the timers.
  /// @param options Node options, forwarded to rclcpp::Node.
  explicit HealthMonitor(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  /// @brief Topic the health snapshot is published on.
  static inline const std::string OUTPUT_TOPIC = "/health_check/health";
  /// @brief Queue depth of the snapshot publisher.
  static constexpr int OUTPUT_QUEUE_DEPTH = 10;

  /// @brief Default trailing window that rate and bandwidth are averaged over,
  /// in seconds.
  static constexpr double DEFAULT_WINDOW_SEC = 5.0;
  /// @brief Default silence after which a bound topic is reported stale, in
  /// seconds.
  static constexpr double DEFAULT_STALE_TIMEOUT_SEC = 2.0;
  /// @brief Default fraction of the expected rate a topic may fall below before
  /// it counts as slow.
  static constexpr double DEFAULT_RATE_TOLERANCE = 0.5;
  /// @brief Default rate that health snapshots are published at, in Hz.
  static constexpr double DEFAULT_PUBLISH_RATE_HZ = 1.0;
  /// @brief Default rate that unbound topics are retried at, in Hz.
  static constexpr double DEFAULT_DISCOVERY_RATE_HZ = 0.5;
  /// @brief Default interface the link counters are read from.
  static inline const std::string DEFAULT_INTERFACE_NAME = "wlan0";
  /// @brief Default seconds between reachability probes.
  static constexpr double DEFAULT_PING_PERIOD_SEC = 5.0;
  /// @brief Default seconds a single reachability probe waits for a reply.
  static constexpr double DEFAULT_PING_TIMEOUT_SEC = 1.0;
  /// @brief Lowest permitted rate tolerance, i.e. no tolerance at all.
  static constexpr double MIN_RATE_TOLERANCE = 0.0;
  /// @brief Highest permitted rate tolerance, i.e. any rate is acceptable.
  static constexpr double MAX_RATE_TOLERANCE = 1.0;

  /// @brief Retries binding any topic monitor that has not found a publisher
  /// yet.
  void _on_discovery_timer();

  /// @brief Builds and publishes one SystemHealth snapshot.
  void _on_publish_timer();

  /// @brief Worst status across every topic and the link.
  /// @param message Snapshot with its topic and link fields already filled in.
  /// @return One of the SystemHealth status constants.
  static std::uint8_t
  _overall_status(const interfaces::msg::SystemHealth &message);

  std::vector<std::unique_ptr<TopicMonitor>> _topic_monitors;
  std::unique_ptr<LinkMonitor> _link_monitor;

  rclcpp::Publisher<interfaces::msg::SystemHealth>::SharedPtr _publisher;
  rclcpp::TimerBase::SharedPtr _discovery_timer;
  rclcpp::TimerBase::SharedPtr _publish_timer;
};

} // namespace health_check
