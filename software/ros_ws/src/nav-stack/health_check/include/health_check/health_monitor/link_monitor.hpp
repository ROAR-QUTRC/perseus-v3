#pragma once

/// @file link_monitor.hpp
/// @brief Network interface counters and base-station reachability.

#include <atomic>
#include <condition_variable>
#include <cstddef>
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
  static bool is_host_safe(const std::string &host);

private:
  /// @brief Longest permitted hostname, per the DNS name length limit.
  static constexpr std::size_t MAX_HOST_LENGTH = 253;

  /// @brief Body of the probe thread: probe, wait, repeat until stopped.
  void _probe_loop();

  /// @brief Runs one ping and records the result.
  void _probe_once();

  /// @brief Reads the counters for the configured interface out of
  /// /proc/net/dev.
  /// @param[out] out Counters, untouched unless the interface was found.
  /// @return True if the interface was present.
  bool _read_counters(interfaces::msg::LinkHealth &out) const;

  std::string _interface_name;
  std::string _ping_host;
  double _ping_period_sec;
  double _ping_timeout_sec;
  bool _is_ping_enabled{false};

  /// @brief Previous counter sample, for turning cumulative byte counts into
  /// rates.
  bool _has_previous_sample{false};
  std::uint64_t _previous_rx_bytes{0};
  std::uint64_t _previous_tx_bytes{0};
  rclcpp::Time _previous_sample_time{0, 0, RCL_ROS_TIME};

  std::thread _probe_thread;
  std::mutex _probe_mutex;
  std::condition_variable _probe_condition;
  bool _is_stop_requested{false};
  bool _is_reachable{false};
  double _rtt_ms{-1.0};
};

} // namespace health_check
