# PROJECT SETUP
cmake_minimum_required(VERSION 3.23)

project(
  mission_bt_server
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
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
                                               "MinSizeRel" "RelWithDebInfo")
endif()
# we always want debug info for stack tracing, so switch to RelWithDebInfo from
# Release
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# COMPILER SETUP
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow)
endif()
if(CMAKE_BUILD_TYPE MATCHES "Rel.*")
  add_compile_options(-Werror)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# INCLUDE DEPENDENCIES
set(THIS_PACKAGE_INCLUDE_DEPENDS rclcpp interfaces std_srvs geometry_msgs
                                 behaviortree_cpp nav2_behavior_tree)

set(THIS_PACKAGE_BUILD_DEPENDS ament_cmake)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

# COMPILE
#
# The BT leaf node is its own shared library, loaded by bare name
# ("request_zone_waypoint_bt_node") through BT.CPP's plugin loader - the same
# convention nav2 uses for its own action/condition/decorator nodes (e.g.
# libnav2_navigate_to_pose_action_bt_node.so), which is what lets
# go_to_zone_waypoint.xml load both from one plugin_lib_names list without
# either one knowing about the other's package.
add_library(request_zone_waypoint_bt_node SHARED
            src/mission_bt_server/request_zone_waypoint_bt_node.cpp)
target_include_directories(request_zone_waypoint_bt_node
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(
  request_zone_waypoint_bt_node
  PUBLIC
  rclcpp
  interfaces
  geometry_msgs
  behaviortree_cpp
  nav2_behavior_tree)
# Without this, BT_REGISTER_NODES expands BT_RegisterNodesFromPlugin as `static`
# (internal linkage, invisible to dlopen/registerFromPlugin) rather than an
# exported symbol - see behaviortree_cpp/bt_factory.h's BTCPP_EXPORT macro.
target_compile_definitions(request_zone_waypoint_bt_node
                           PRIVATE BT_PLUGIN_EXPORT)

add_executable(
  mission_bt_server src/mission_bt_server/mission_bt_server.cpp
                    src/mission_bt_server/mission_bt_server_node.cpp)
target_include_directories(mission_bt_server
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(mission_bt_server PUBLIC
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

# INSTALL (ROS)
#
# The BT plugin goes straight to lib/, not lib/${PROJECT_NAME}/: BT.CPP's
# plugin loader resolves a bare name through the OS library search path
# (LD_LIBRARY_PATH, which every package's lib/ is on via its ament hook), the
# same place nav2's own plugin .so's install to.
install(
  TARGETS request_zone_waypoint_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS mission_bt_server DESTINATION lib/${PROJECT_NAME})

# No config/ or launch/ here on purpose: like arena_server, the launch file
# and the BT xml live in autonomy_bringup alongside the other stack configs.

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ROS FINALISATION
ament_package()
