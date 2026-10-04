/// @file height_crop_node.cpp
/// @brief Entry point for the height crop node.

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "sensors/height_crop/height_crop.hpp"

/// @brief Spins the height crop until the node is shut down.
/// @param argc Argument count passed to ROS for command line parameter parsing.
/// @param argv Argument values passed to ROS for command line parameter
/// parsing.
/// @return Zero on a clean shutdown.
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sensors::HeightCrop>());
    rclcpp::shutdown();
    return 0;
}
