# cloud_mesher lives in its own directory scope, and that is not cosmetic.
#
# Two things here reach further than they look. find_package(PCL) calls
# link_directories(), which applies to every target defined afterwards in the
# same scope, and it reports /usr/include among its include directories -- host
# glibc headers that this toolchain must not see, or pthread_cond_t resolves to
# the wrong type and <mutex> stops compiling. Run from the package's top-level
# CMakeLists, both reached targets with no connection to PCL: topic_remapper
# failed to link against libnl, having never asked for it.
#
# pcl_conversions is scoped the same way and for the same reason. Listing it in
# the package-wide dependency list hands every target the whole of PCL, VTK
# included.

list(APPEND CMAKE_C_IMPLICIT_INCLUDE_DIRECTORIES /usr/include)
list(APPEND CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES /usr/include)

find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common features kdtree search surface)

add_executable(cloud_mesher cloud_mesher.cpp)
ament_target_dependencies(cloud_mesher PUBLIC rclcpp sensor_msgs
                          visualization_msgs pcl_conversions)
target_include_directories(cloud_mesher SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
# The component libraries actually used, not the aggregate ${PCL_LIBRARIES}:
# that resolves to every PCL module and their dependencies, VTK included, and
# VTK then wants libnl and libva at link time for rendering code nothing here
# calls.
target_link_libraries(cloud_mesher PUBLIC pcl_common pcl_features pcl_kdtree
                                          pcl_search pcl_surface)
# libpcl_surface links VTK, and VTK needs libnl and libva. Nothing here calls
# any of it -- those belong to VTK's rendering code -- but the linker still
# tries to resolve them, and in a colcon build they are not on the link path.
# --allow-shlib-undefined leaves symbols belonging to a transitive shared
# library alone; the loader resolves them through VTK's own rpath at runtime.
# Symbols this node actually references are still checked.
target_link_options(cloud_mesher PRIVATE -Wl,--allow-shlib-undefined)

# PCL's headers are not -Wall -Wextra -Wpedantic clean and this package builds
# with -Werror. Scoped to this target so everything else keeps its warnings
# fatal.
target_compile_options(cloud_mesher PRIVATE -Wno-error)

install(TARGETS cloud_mesher DESTINATION lib/${PROJECT_NAME})
