# PROJECT SETUP
cmake_minimum_required(VERSION 3.23)

project(
  payloads
  VERSION 0.0.1
  LANGUAGES CXX)

# credit https://www.kitware.com/cmake-and-the-default-build-type/
set(default_build_type "Debug")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(
    STATUS
      "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE
      "${default_build_type}"
      CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
                                               "MinSizeRel" "RelWithDebInfo")
endif()
# we always want debug info for stack tracing, so switch to RelWithDebInfo from
# Release
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
endif()
# Add -Werror flag for release builds
if(CMAKE_BUILD_TYPE MATCHES "Rel.*")
  add_compile_options(-Werror)
endif()

# Set the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# INCLUDE DEPENDENCIES

# public dependencies (which should propagate)
set(THIS_PACKAGE_INCLUDE_DEPENDS rclcpp sensor_msgs actuator_msgs)

# dependencies of the ros2_control hardware plugin only
set(HARDWARE_PLUGIN_DEPENDS
    dynamixel_sdk
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    transmission_interface
    hector_transmission_interface
    hector_transmission_interface_msgs)

# packages which must be present but aren't linked against
set(THIS_PACKAGE_BUILD_DEPENDS backward_ros ament_cmake)

foreach(Dependency IN ITEMS ${HARDWARE_PLUGIN_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(hi_can_raw QUIET)
if(NOT hi_can_raw_FOUND)
  message(STATUS "hi-can-raw not found, adding subdirectory...")
  message(
    STATUS "Attempting to add hi-can-raw from: ../../../shared/hi-can-raw")
  add_subdirectory(../../../shared/hi-can-raw hi-can-raw)
endif()

# COMPILE (Executables)

# Find all source files for the executables
file(GLOB_RECURSE ARM_DRIVER_CODE_SOURCES src/arm_driver/*.cpp)
file(GLOB_RECURSE BUCKET_CONTROLLER_CODE_SOURCES src/bucket_controller/*.cpp)
file(GLOB_RECURSE BUCKET_DRIVER_CODE_SOURCES src/bucket_driver/*.cpp)

# Add exec targets
add_executable(arm_driver ${ARM_DRIVER_CODE_SOURCES})
add_executable(bucket_controller ${BUCKET_CONTROLLER_CODE_SOURCES})
add_executable(bucket_driver ${BUCKET_DRIVER_CODE_SOURCES})

# Add include dirs
target_include_directories(arm_driver
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(bucket_controller
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(bucket_driver
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add dependencies for the executables
ament_target_dependencies(arm_driver PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
                          sensor_msgs)
ament_target_dependencies(
  bucket_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS} actuator_msgs
  sensor_msgs)
ament_target_dependencies(bucket_driver PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}
                          actuator_msgs)

target_link_libraries(bucket_driver PUBLIC hi_can_raw)

# COMPILE (ros2_control hardware plugin)

# Dynamixel bus SystemInterface, loaded by controller_manager when
# use_mock_hardware is false. Built as a shared library so pluginlib can dlopen
# it; the library name must match <library path=...> in payloads_hardware.xml.
add_library(
  dynamixel_servos SHARED src/dynamixel_servos/dynamixel_servos.cpp
                          src/dynamixel_servos/dynamixel_controller.cpp)

target_include_directories(
  dynamixel_servos PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                          $<INSTALL_INTERFACE:include>)

ament_target_dependencies(dynamixel_servos PUBLIC ${HARDWARE_PLUGIN_DEPENDS})

install(
  TARGETS dynamixel_servos
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/dynamixel_servos DESTINATION include)

# Install ROS Executable
install(TARGETS arm_driver DESTINATION lib/${PROJECT_NAME})
install(TARGETS bucket_controller DESTINATION lib/${PROJECT_NAME})
install(TARGETS bucket_driver DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

install(
  PROGRAMS scripts/gripper_velocity_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gripper_velocity_bridge)

install(
  PROGRAMS scripts/wrist_velocity_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wrist_velocity_bridge)

install(
  PROGRAMS scripts/keyboard_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_control)

install(
  PROGRAMS scripts/arm_home.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME arm_home)

# EXPORT PLUGINLIB INFORMATION

# ROS2 Control hardware interface plugin
pluginlib_export_plugin_description_file(hardware_interface
                                         payloads_hardware.xml)

# ROS LIBRARY POST-INSTALL
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(${HARDWARE_PLUGIN_DEPENDS})

ament_package()
