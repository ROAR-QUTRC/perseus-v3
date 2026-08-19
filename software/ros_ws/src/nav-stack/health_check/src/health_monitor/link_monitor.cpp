/// @file link_monitor.cpp
/// @brief Implementation of the network interface and reachability monitoring.

#include "health_check/health_monitor/link_monitor.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <utility>

namespace health_check {
namespace {

/// @brief Fields of /proc/net/dev in order, after the "iface:" prefix is
/// stripped. Only the ones the message carries are pulled out by index.
constexpr int kRxBytesField = 0;
constexpr int kRxErrorsField = 2;
constexpr int kRxDroppedField = 3;
constexpr int kTxBytesField = 8;
constexpr int kTxErrorsField = 10;
constexpr int kTxDroppedField = 11;
constexpr int kFieldCount = 16;

} // namespace

LinkMonitor::LinkMonitor(std::string interface_name, std::string ping_host,
                         double ping_period_sec, double ping_timeout_sec)
    : interface_name_(std::move(interface_name)),
      ping_host_(std::move(ping_host)), ping_period_sec_(ping_period_sec),
      ping_timeout_sec_(ping_timeout_sec) {}

LinkMonitor::~LinkMonitor() {
  {
    const std::lock_guard<std::mutex> lock(probe_mutex_);
    stop_requested_ = true;
  }
  probe_cv_.notify_all();
  if (probe_thread_.joinable()) {
    probe_thread_.join();
  }
}

bool LinkMonitor::isHostSafe(const std::string &host) {
  if (host.empty() || host.size() > 253) {
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
  if (ping_host_.empty()) {
    RCLCPP_INFO(logger,
                "No ping host configured; reporting link counters only.");
    return;
  }
  if (!isHostSafe(ping_host_)) {
    RCLCPP_ERROR(
        logger,
        "Refusing to probe ping host '%s': only letters, digits, '.', ':', '-' "
        "and '_' are accepted. Reachability reporting is disabled.",
        ping_host_.c_str());
    return;
  }

  ping_enabled_ = true;
  probe_thread_ = std::thread(&LinkMonitor::probeLoop, this);
}

void LinkMonitor::probeLoop() {
  for (;;) {
    probeOnce();

    std::unique_lock<std::mutex> lock(probe_mutex_);
    const auto period = std::chrono::duration<double>(ping_period_sec_);
    probe_cv_.wait_for(lock, period, [this] { return stop_requested_; });
    if (stop_requested_) {
      return;
    }
  }
}

void LinkMonitor::probeOnce() {
  /*
    -n skips reverse DNS, which would otherwise add its own timeout on a network
    that has lost its resolver -- exactly the situation this is trying to
    measure. The host has already been checked against isHostSafe(), so it
    cannot introduce shell syntax here.
  */
  std::ostringstream command;
  command << "ping -n -c 1 -W " << static_cast<int>(ping_timeout_sec_ + 0.5)
          << " " << ping_host_ << " 2>/dev/null";

  bool reachable = false;
  double rtt_ms = -1.0;

  std::FILE *pipe = popen(command.str().c_str(), "r");
  if (pipe != nullptr) {
    std::array<char, 256> buffer{};
    while (std::fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) !=
           nullptr) {
      const std::string line(buffer.data());
      const auto marker = line.find("time=");
      if (marker == std::string::npos) {
        continue;
      }
      try {
        rtt_ms = std::stod(line.substr(marker + 5));
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

  const std::lock_guard<std::mutex> lock(probe_mutex_);
  reachable_ = reachable;
  // Hold the last good RTT rather than zeroing it, so "no reply" stays
  // distinguishable.
  if (reachable) {
    rtt_ms_ = rtt_ms;
  } else {
    rtt_ms_ = -1.0;
  }
}

bool LinkMonitor::readCounters(interfaces::msg::LinkHealth &out) const {
  std::ifstream proc("/proc/net/dev");
  if (!proc) {
    return false;
  }

  std::string line;
  // Two header lines precede the per-interface rows.
  std::getline(proc, line);
  std::getline(proc, line);

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
    if (name != interface_name_) {
      continue;
    }

    std::istringstream fields(line.substr(colon + 1));
    std::array<std::uint64_t, kFieldCount> values{};
    for (int i = 0; i < kFieldCount; ++i) {
      if (!(fields >> values[static_cast<std::size_t>(i)])) {
        return false;
      }
    }

    out.rx_errors = values[kRxErrorsField];
    out.tx_errors = values[kTxErrorsField];
    out.rx_dropped = values[kRxDroppedField];
    out.tx_dropped = values[kTxDroppedField];
    // Byte totals are returned through the rate fields and converted by the
    // caller.
    out.rx_bytes_per_sec = static_cast<double>(values[kRxBytesField]);
    out.tx_bytes_per_sec = static_cast<double>(values[kTxBytesField]);
    return true;
  }

  return false;
}

interfaces::msg::LinkHealth LinkMonitor::report(const rclcpp::Time &now) {
  interfaces::msg::LinkHealth health;
  health.interface_name = interface_name_;
  health.ping_enabled = ping_enabled_;
  health.ping_host = ping_host_;

  {
    const std::lock_guard<std::mutex> lock(probe_mutex_);
    health.reachable = ping_enabled_ && reachable_;
    health.rtt_ms = ping_enabled_ ? rtt_ms_ : -1.0;
  }

  health.interface_present = readCounters(health);
  if (!health.interface_present) {
    health.rx_bytes_per_sec = 0.0;
    health.tx_bytes_per_sec = 0.0;
    return health;
  }

  // readCounters() leaves cumulative totals in the rate fields; difference them
  // here.
  const auto rx_total = static_cast<std::uint64_t>(health.rx_bytes_per_sec);
  const auto tx_total = static_cast<std::uint64_t>(health.tx_bytes_per_sec);

  if (have_previous_) {
    const double elapsed = (now - previous_sample_time_).seconds();
    if (elapsed > 0.0) {
      /*
        Counters are monotonic in practice but wrap on 32-bit kernels and reset
        if the interface is reconfigured. Clamp instead of reporting a huge
        negative rate.
      */
      const std::uint64_t rx_delta =
          rx_total >= previous_rx_bytes_ ? rx_total - previous_rx_bytes_ : 0;
      const std::uint64_t tx_delta =
          tx_total >= previous_tx_bytes_ ? tx_total - previous_tx_bytes_ : 0;
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

  previous_rx_bytes_ = rx_total;
  previous_tx_bytes_ = tx_total;
  previous_sample_time_ = now;
  have_previous_ = true;

  return health;
}

} // namespace health_check
