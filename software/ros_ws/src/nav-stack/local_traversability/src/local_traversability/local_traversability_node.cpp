/// @file local_traversability_node.cpp
/// @brief Entry point for the local traversability node.

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "local_traversability/local_traversability/local_traversability.hpp"

/// @brief Spins the local traversability node until it is shut down.
/// @param argc Argument count passed to ROS for command line parameter parsing.
/// @param argv Argument values passed to ROS for command line parameter
/// parsing.
/// @return Zero on a clean shutdown.
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<local_traversability::LocalTraversability>());
    rclcpp::shutdown();
    return 0;
}
