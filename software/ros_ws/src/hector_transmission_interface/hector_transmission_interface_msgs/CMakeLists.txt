cmake_minimum_required(VERSION 3.8)
project(hector_transmission_interface_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} srv/AdjustTransmissionOffsets.srv
                           srv/FlipByPi.srv DEPENDENCIES sensor_msgs)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
