/// @file rock_detector_node.cpp
/// @brief Standalone entry point for the rock detection node.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "perseus_vision/rock_detector/rock_detector.hpp"

/// @brief Spins the rock detector until ROS 2 shuts down.
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Instantiate RockDetector, matching the corrected header and component class.
    rclcpp::spin(std::make_shared<perseus_vision::RockDetector>());

    rclcpp::shutdown();
    return 0;
}
