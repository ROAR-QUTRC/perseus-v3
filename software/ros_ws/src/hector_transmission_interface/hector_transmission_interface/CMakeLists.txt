cmake_minimum_required(VERSION 3.16)
project(hector_transmission_interface LANGUAGES CXX)

# Standard Compiler and Export Settings
find_package(ros2_control_cmake QUIET)
if(ros2_control_cmake_FOUND)
  set_compiler_options()
  export_windows_symbols()
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    pluginlib rclcpp transmission_interface controller_manager_msgs
    hector_transmission_interface_msgs sensor_msgs)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gen_version_h REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

# --- Main Library ---
add_library(${PROJECT_NAME} SHARED src/adjustable_offset_transmission_loader.cpp
                                   src/adjustable_offset_manager.cpp)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Pluginlib Export
pluginlib_export_plugin_description_file(${PROJECT_NAME}
                                         ros2_control_plugins.xml)

# --- Testing Section ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_adjustable_offset_transmission
                    test/test_adjustable_offset_transmission.cpp)
    target_link_libraries(test_adjustable_offset_transmission ${PROJECT_NAME}
                          ${rclcpp_LIBRARIES})
    ament_target_dependencies(test_adjustable_offset_transmission rclcpp
                              transmission_interface)
  endif()
endif()


# --- Installation & Export ---
install(DIRECTORY include/ DESTINATION include/)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
ament_generate_version_header(${PROJECT_NAME})
