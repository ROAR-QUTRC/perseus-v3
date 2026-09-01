# PROJECT SETUP
cmake_minimum_required(VERSION 3.23)

project(
  arena_server
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
set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    geometry_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    interfaces)

set(THIS_PACKAGE_BUILD_DEPENDS ament_cmake)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

# nlohmann/json is header-only and already on the include path in this
# environment, so there is deliberately no find_package for it: there is no
# nlohmann entry on CMAKE_PREFIX_PATH here, so find_package(nlohmann_json) fails
# even though the header compiles. That works but is implicit -- if this ever
# builds somewhere the header is not on the default include path, add
# nlohmann_json to devenv.nix and a find_package here rather than hunting the
# missing include.

# COMPILE
#
# The JSON reader is a library, not just a shared source file, because there are
# now three consumers: the robot-side server, the base-station marker node, and
# the RViz minimap panel in rviz_plugins. Exporting it is what keeps "one
# parser" true across package boundaries - a second implementation in the panel
# would be free to disagree about the layout, silently.
add_library(arena_layout SHARED src/arena_server/arena_layout.cpp)
target_include_directories(
  arena_layout PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                      $<INSTALL_INTERFACE:include>)

add_executable(arena_server src/arena_server/arena_server.cpp
                            src/arena_server/arena_server_node.cpp)
target_include_directories(arena_server
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(arena_server PUBLIC arena_layout)
ament_target_dependencies(arena_server PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(arena_minimap src/arena_server/arena_minimap_node.cpp)
target_include_directories(arena_minimap
                           PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(arena_minimap PUBLIC arena_layout)
ament_target_dependencies(arena_minimap PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

# INSTALL (ROS)
install(TARGETS arena_server arena_minimap DESTINATION lib/${PROJECT_NAME})
install(
  TARGETS arena_layout
  EXPORT export_arena_layout
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/)

ament_export_targets(export_arena_layout HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(arena_layout)

# No config/ or launch/ here on purpose: the arena layout and the launch file
# live in autonomy_bringup, alongside the other stack configs, so this package
# is just the node.

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ROS FINALISATION
ament_package()
