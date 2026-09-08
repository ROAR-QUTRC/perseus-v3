/// @file health_monitor_node.cpp
/// @brief Entry point for the health monitor node.

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "health_check/health_monitor/health_monitor.hpp"

/// @brief Spins the health monitor until the node is shut down.
/// @param argc Argument count passed to ROS for command line parameter parsing.
/// @param argv Argument values passed to ROS for command line parameter
/// parsing.
/// @return Zero on a clean shutdown.
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<health_check::HealthMonitor>());
  rclcpp::shutdown();
  return 0;
}
