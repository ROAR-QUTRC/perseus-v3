/// @file mission_bt_server_node.cpp
/// @brief Entry point for mission_bt_server.

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "mission_bt_server/mission_bt_server/mission_bt_server.hpp"

/// @brief Spins mission_bt_server until shut down.
///
/// Multi-threaded, like arena_server: the two mission services block for the
/// whole navigation on a reentrant callback group, and on a single-threaded
/// executor that would stall the very executor the BT's own service/action
/// clients need spun.
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mission_bt_server::MissionBtServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
