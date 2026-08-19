/// @file health_monitor.hpp
/// @brief Node that aggregates topic and link health and publishes it.

#ifndef HEALTH_CHECK__HEALTH_MONITOR__HEALTH_MONITOR_HPP_
#define HEALTH_CHECK__HEALTH_MONITOR__HEALTH_MONITOR_HPP_

#include <memory>
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
  /// @brief Retries binding any topic monitor that has not found a publisher
  /// yet.
  void onDiscoveryTimer();

  /// @brief Builds and publishes one SystemHealth snapshot.
  void onPublishTimer();

  /// @brief Worst status across every topic and the link.
  /// @param message Snapshot with its topic and link fields already filled in.
  /// @return One of the SystemHealth status constants.
  static std::uint8_t
  overallStatus(const interfaces::msg::SystemHealth &message);

  std::vector<std::unique_ptr<TopicMonitor>> topic_monitors_;
  std::unique_ptr<LinkMonitor> link_monitor_;

  rclcpp::Publisher<interfaces::msg::SystemHealth>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr discovery_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

} // namespace health_check

#endif // HEALTH_CHECK__HEALTH_MONITOR__HEALTH_MONITOR_HPP_
