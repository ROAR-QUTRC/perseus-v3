/// @file orb_slam_odometry_node.cpp
/// @brief Entry point for running OrbSlamOdometry standalone with `ros2 run`.

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "vision/orb_slam_odometry/orb_slam_odometry.hpp"

/// @brief Spins the node until shutdown.
/// @param argc Argument count, forwarded to ROS.
/// @param argv Argument values, forwarded to ROS.
/// @return Zero on a clean shutdown.
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vision::OrbSlamOdometry>());
    rclcpp::shutdown();
    return 0;
}
