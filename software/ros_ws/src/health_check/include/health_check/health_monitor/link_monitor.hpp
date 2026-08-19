/// @file link_monitor.hpp
/// @brief Network interface counters and base-station reachability.

#ifndef HEALTH_CHECK__HEALTH_MONITOR__LINK_MONITOR_HPP_
#define HEALTH_CHECK__HEALTH_MONITOR__LINK_MONITOR_HPP_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include <interfaces/msg/link_health.hpp>
#include <rclcpp/rclcpp.hpp>

namespace health_check {

/// @brief Reports throughput and errors on one network interface, and
/// optionally whether a base station is still answering.
///
/// This is the half of the health picture that ROS-level topic rates cannot
/// show. A wifi link shedding packets will often leave topic rates looking fine
/// right up until it doesn't, whereas the drop counters here start moving
/// early.
///
/// The reachability probe runs on its own thread rather than in the publish
/// cycle, because a ping to a host that has gone away blocks for the timeout
/// and would otherwise stall the executor for as long as it takes.
class LinkMonitor {
public:
  /// @brief Creates a link monitor.
  /// @param interface_name Interface to read from /proc/net/dev, e.g. "wlan0".
  /// @param ping_host Host to probe, or empty to disable probing entirely.
  /// @param ping_period_sec Seconds between probes.
  /// @param ping_timeout_sec How long a single probe waits for a reply.
  LinkMonitor(std::string interface_name, std::string ping_host,
              double ping_period_sec, double ping_timeout_sec);

  /// @brief Stops the probe thread and joins it.
  ~LinkMonitor();

  LinkMonitor(const LinkMonitor &) = delete;
  LinkMonitor &operator=(const LinkMonitor &) = delete;

  /// @brief Starts the probe thread, if a host was configured.
  /// @param logger Logger used to report a rejected host.
  void start(const rclcpp::Logger &logger);

  /// @brief Samples the interface counters and folds in the latest probe
  /// result.
  /// @param now Time of this sample, used to turn cumulative counters into
  /// rates.
  /// @return Link health as of now.
  interfaces::msg::LinkHealth report(const rclcpp::Time &now);

  /// @brief Whether a hostname is safe to hand to the ping command.
  ///
  /// The host arrives as a ROS parameter and ends up in a command line, so
  /// anything outside the character set a hostname or IP literal can
  /// legitimately contain is rejected rather than escaped. Exposed for testing.
  ///
  /// @param host Candidate hostname or address.
  /// @return True if the host is safe to use.
  static bool isHostSafe(const std::string &host);

private:
  /// @brief Body of the probe thread: probe, wait, repeat until stopped.
  void probeLoop();

  /// @brief Runs one ping and records the result.
  void probeOnce();

  /// @brief Reads the counters for the configured interface out of
  /// /proc/net/dev.
  /// @param[out] out Counters, untouched unless the interface was found.
  /// @return True if the interface was present.
  bool readCounters(interfaces::msg::LinkHealth &out) const;

  std::string interface_name_;
  std::string ping_host_;
  double ping_period_sec_;
  double ping_timeout_sec_;
  bool ping_enabled_{false};

  /// @brief Previous counter sample, for turning cumulative byte counts into
  /// rates.
  bool have_previous_{false};
  std::uint64_t previous_rx_bytes_{0};
  std::uint64_t previous_tx_bytes_{0};
  rclcpp::Time previous_sample_time_{0, 0, RCL_ROS_TIME};

  std::thread probe_thread_;
  std::mutex probe_mutex_;
  std::condition_variable probe_cv_;
  bool stop_requested_{false};
  bool reachable_{false};
  double rtt_ms_{-1.0};
};

} // namespace health_check

#endif // HEALTH_CHECK__HEALTH_MONITOR__LINK_MONITOR_HPP_
