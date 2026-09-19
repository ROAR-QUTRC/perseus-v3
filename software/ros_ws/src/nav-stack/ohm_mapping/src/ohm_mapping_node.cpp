// Copyright 2026 ROAR QUTRC
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "ohm_mapping/ohm_mapping.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ohm_mapping::OhmMapping>());
    rclcpp::shutdown();
    return 0;
}
