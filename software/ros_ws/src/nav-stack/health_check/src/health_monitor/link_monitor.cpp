/// @file link_monitor.cpp
/// @brief Implementation of the network interface and reachability monitoring.

#include "health_check/health_monitor/link_monitor.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string_view>
#include <utility>

namespace health_check {
namespace {

/// @brief Fields of /proc/net/dev in order, after the "iface:" prefix is
/// stripped. Only the ones the message carries are pulled out by index.
constexpr int RX_BYTES_FIELD = 0;
constexpr int RX_ERRORS_FIELD = 2;
constexpr int RX_DROPPED_FIELD = 3;
constexpr int TX_BYTES_FIELD = 8;
constexpr int TX_ERRORS_FIELD = 10;
constexpr int TX_DROPPED_FIELD = 11;
constexpr int FIELD_COUNT = 16;

/// @brief Number of header lines in /proc/net/dev before the per-interface
/// rows.
constexpr int HEADER_LINE_COUNT = 2;

/// @brief Marker preceding the round-trip time in a ping reply line.
constexpr std::string_view RTT_MARKER = "time=";

/// @brief Size of the buffer a single line of ping output is read into.
constexpr std::size_t PING_OUTPUT_LINE_LENGTH = 256;

} // namespace

LinkMonitor::LinkMonitor(std::string interface_name, std::string ping_host,
                         double ping_period_sec, double ping_timeout_sec)
    : _interface_name(std::move(interface_name)),
      _ping_host(std::move(ping_host)), _ping_period_sec(ping_period_sec),
      _ping_timeout_sec(ping_timeout_sec) {}

LinkMonitor::~LinkMonitor() {
  {
    const std::lock_guard<std::mutex> lock(_probe_mutex);
    _is_stop_requested = true;
  }
  _probe_condition.notify_all();
  if (_probe_thread.joinable()) {
    _probe_thread.join();
  }
}

bool LinkMonitor::is_host_safe(const std::string &host) {
  if (host.empty() || host.size() > MAX_HOST_LENGTH) {
    return false;
  }
  // A leading dash would be read as an option by the ping command.
  if (host.front() == '-') {
    return false;
  }
  return std::all_of(host.begin(), host.end(), [](unsigned char c) {
    return std::isalnum(c) != 0 || c == '.' || c == ':' || c == '-' || c == '_';
  });
}

void LinkMonitor::start(const rclcpp::Logger &logger) {
  if (_ping_host.empty()) {
    RCLCPP_INFO(logger,
                "No ping host configured; reporting link counters only.");
    return;
  }
  if (!is_host_safe(_ping_host)) {
    RCLCPP_ERROR(
        logger,
        "Refusing to probe ping host '%s': only letters, digits, '.', ':', '-' "
        "and '_' are accepted. Reachability reporting is disabled.",
        _ping_host.c_str());
    return;
  }

  _is_ping_enabled = true;
  _probe_thread = std::thread(&LinkMonitor::_probe_loop, this);
}

void LinkMonitor::_probe_loop() {
  for (;;) {
    _probe_once();

    std::unique_lock<std::mutex> lock(_probe_mutex);
    const auto period = std::chrono::duration<double>(_ping_period_sec);
    _probe_condition.wait_for(lock, period,
                              [this] { return _is_stop_requested; });
    if (_is_stop_requested) {
      return;
    }
  }
}

void LinkMonitor::_probe_once() {
  /*
    -n skips reverse DNS, which would otherwise add its own timeout on a network
    that has lost its resolver -- exactly the situation this is trying to
    measure. The host has already been checked against is_host_safe(), so it
    cannot introduce shell syntax here.
  */
  std::ostringstream command;
  command << "ping -n -c 1 -W "
          << static_cast<int>(std::lround(_ping_timeout_sec)) << " "
          << _ping_host << " 2>/dev/null";

  bool reachable = false;
  double rtt_ms = -1.0;

  std::FILE *pipe = popen(command.str().c_str(), "r");
  if (pipe != nullptr) {
    std::array<char, PING_OUTPUT_LINE_LENGTH> buffer{};
    while (std::fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) !=
           nullptr) {
      const std::string line(buffer.data());
      const auto marker = line.find(RTT_MARKER);
      if (marker == std::string::npos) {
        continue;
      }
      try {
        rtt_ms = std::stod(line.substr(marker + RTT_MARKER.size()));
        reachable = true;
      } catch (const std::exception &) {
        // A reply we could not parse still tells us the host answered.
        reachable = true;
      }
      break;
    }
    // Exit status is authoritative: a non-zero status means no reply, whatever
    // we parsed.
    if (pclose(pipe) != 0) {
      reachable = false;
    }
  }

  const std::lock_guard<std::mutex> lock(_probe_mutex);
  _is_reachable = reachable;
  // Hold the last good RTT rather than zeroing it, so "no reply" stays
  // distinguishable.
  if (reachable) {
    _rtt_ms = rtt_ms;
  } else {
    _rtt_ms = -1.0;
  }
}

bool LinkMonitor::_read_counters(interfaces::msg::LinkHealth &out) const {
  std::ifstream proc("/proc/net/dev");
  if (!proc) {
    return false;
  }

  std::string line;
  for (int i = 0; i < HEADER_LINE_COUNT; ++i) {
    std::getline(proc, line);
  }

  while (std::getline(proc, line)) {
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }

    std::string name = line.substr(0, colon);
    // Rows are right-aligned, so the interface name carries leading spaces.
    const auto first = name.find_first_not_of(" \t");
    if (first == std::string::npos) {
      continue;
    }
    name = name.substr(first);
    if (name != _interface_name) {
      continue;
    }

    std::istringstream fields(line.substr(colon + 1));
    std::array<std::uint64_t, FIELD_COUNT> values{};
    for (int i = 0; i < FIELD_COUNT; ++i) {
      if (!(fields >> values[static_cast<std::size_t>(i)])) {
        return false;
      }
    }

    out.rx_errors = values[RX_ERRORS_FIELD];
    out.tx_errors = values[TX_ERRORS_FIELD];
    out.rx_dropped = values[RX_DROPPED_FIELD];
    out.tx_dropped = values[TX_DROPPED_FIELD];
    // Byte totals are returned through the rate fields and converted by the
    // caller.
    out.rx_bytes_per_sec = static_cast<double>(values[RX_BYTES_FIELD]);
    out.tx_bytes_per_sec = static_cast<double>(values[TX_BYTES_FIELD]);
    return true;
  }

  return false;
}

interfaces::msg::LinkHealth LinkMonitor::report(const rclcpp::Time &now) {
  interfaces::msg::LinkHealth health;
  health.interface_name = _interface_name;
  health.is_ping_enabled = _is_ping_enabled;
  health.ping_host = _ping_host;

  {
    const std::lock_guard<std::mutex> lock(_probe_mutex);
    health.is_reachable = _is_ping_enabled && _is_reachable;
    health.rtt_ms = _is_ping_enabled ? _rtt_ms : -1.0;
  }

  health.is_interface_present = _read_counters(health);
  if (!health.is_interface_present) {
    health.rx_bytes_per_sec = 0.0;
    health.tx_bytes_per_sec = 0.0;
    return health;
  }

  // _read_counters() leaves cumulative totals in the rate fields; difference
  // them here.
  const auto rx_total = static_cast<std::uint64_t>(health.rx_bytes_per_sec);
  const auto tx_total = static_cast<std::uint64_t>(health.tx_bytes_per_sec);

  if (_has_previous_sample) {
    const double elapsed = (now - _previous_sample_time).seconds();
    if (elapsed > 0.0) {
      /*
        Counters are monotonic in practice but wrap on 32-bit kernels and reset
        if the interface is reconfigured. Clamp instead of reporting a huge
        negative rate.
      */
      const std::uint64_t rx_delta =
          rx_total >= _previous_rx_bytes ? rx_total - _previous_rx_bytes : 0;
      const std::uint64_t tx_delta =
          tx_total >= _previous_tx_bytes ? tx_total - _previous_tx_bytes : 0;
      health.rx_bytes_per_sec = static_cast<double>(rx_delta) / elapsed;
      health.tx_bytes_per_sec = static_cast<double>(tx_delta) / elapsed;
    } else {
      health.rx_bytes_per_sec = 0.0;
      health.tx_bytes_per_sec = 0.0;
    }
  } else {
    // Nothing to difference against on the first sample.
    health.rx_bytes_per_sec = 0.0;
    health.tx_bytes_per_sec = 0.0;
  }

  _previous_rx_bytes = rx_total;
  _previous_tx_bytes = tx_total;
  _previous_sample_time = now;
  _has_previous_sample = true;

  return health;
}

} // namespace health_check
