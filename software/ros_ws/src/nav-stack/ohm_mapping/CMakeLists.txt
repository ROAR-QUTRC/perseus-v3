# PROJECT SETUP
cmake_minimum_required(VERSION 3.23)

project(
  ohm_mapping
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
# Add -Werror flag for release builds
if(CMAKE_BUILD_TYPE MATCHES "Rel.*")
  add_compile_options(-Werror)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# INCLUDE DEPENDENCIES
set(THIS_PACKAGE_INCLUDE_DEPENDS rclcpp sensor_msgs nav_msgs geometry_msgs tf2
                                 tf2_ros)

set(THIS_PACKAGE_BUILD_DEPENDS ament_cmake)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# CSIRO OHM. Supplied by nix/extra-packages/patches/ohm rather than by rosdep --
# it is plain CMake with no ROS content at all, so it is packaged as an ordinary
# library and found here by its own exported config (share/ohm/cmake).
#
# The imported targets are used directly rather than the OHM_LIBRARIES variable
# that ohm-config.cmake assembles, because that variable is missing ohmheightmap:
# the config guards register_target(ohm::ohmheightmap ...) behind
# OHM_FEATURE_HEIGHTMAP_IMAGE instead of OHM_FEATURE_HEIGHTMAP, and never sets
# OHM_FEATURE_HEIGHTMAP at all. The install(EXPORT) targets file defines the
# target regardless, so linking it by name works and sidesteps the bug.
find_package(ohm REQUIRED)

# COMPILE
add_executable(ohm_mapping src/ohm_mapping.cpp src/ohm_mapping_node.cpp)

target_include_directories(ohm_mapping
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

ament_target_dependencies(ohm_mapping PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# OHM's headers arrive as SYSTEM includes because these are IMPORTED targets,
# which matters: -Werror is on for release builds above and OHM is C++14-era code
# that does not compile clean under -Wpedantic.
target_link_libraries(ohm_mapping PUBLIC ohm::ohm ohm::ohmheightmap
                                         ohm::ohmutil)

# INSTALL (ROS)
install(TARGETS ohm_mapping DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ROS FINALISATION
ament_package()
