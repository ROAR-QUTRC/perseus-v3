# PROJECT SETUP
cmake_minimum_required(VERSION 3.23)

project(
  health_check
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

# COMPILER SETUP
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
set(THIS_PACKAGE_INCLUDE_DEPENDS rclcpp std_msgs interfaces rviz_common
                                 pluginlib)

# packages which must be present but aren't linked against
set(THIS_PACKAGE_BUILD_DEPENDS ament_cmake)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Qt5 REQUIRED COMPONENTS Widgets)

# the RViz panel derives from QObject, so moc has to run over its header
set(CMAKE_AUTOMOC ON)

include_directories(include)

# COMPILE (Executables)
add_executable(
  health_monitor
  src/health_monitor/health_monitor.cpp
  src/health_monitor/health_monitor_node.cpp
  src/health_monitor/link_monitor.cpp src/health_monitor/topic_monitor.cpp)

target_include_directories(health_monitor
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_target_dependencies(health_monitor PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# COMPILE (RViz panel plugin)
add_library(
  health_check_panel SHARED src/rviz/topic_health_panel.cpp
                            include/health_check/rviz/topic_health_panel.hpp)

target_include_directories(health_check_panel
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(health_check_panel Qt5::Widgets)

ament_target_dependencies(health_check_panel ${THIS_PACKAGE_INCLUDE_DEPENDS})

# tells pluginlib where to find the plugin manifest
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# INSTALL (ROS)
install(TARGETS health_monitor DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS health_check_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})

ament_export_libraries(health_check_panel)

install(DIRECTORY include/ DESTINATION include/)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ROS FINALISATION
ament_package()
