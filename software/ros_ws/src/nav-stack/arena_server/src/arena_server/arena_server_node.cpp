/// @file arena_server_node.cpp
/// @brief Entry point for the arena server node.

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "arena_server/arena_server/arena_server.hpp"

/// @brief Spins the arena server until it is shut down.
///
/// Uses a multi-threaded executor deliberately: /arena/localise blocks while it
/// waits for the lazy camera to produce frames, and on a single-threaded
/// executor that wait would stall the very subscription it depends on.
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<arena_server::ArenaServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
