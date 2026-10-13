# Replacement top-level build for ORB-SLAM3, used by ./default.nix instead of upstream's.
#
# Upstream builds in-tree with build.sh: DBoW2 and g2o as separate shared libraries in
# Thirdparty/*/lib, the core library in lib/, and no install rules at all, so nothing
# downstream can find_package() it. This builds the same sources into one
# libORB_SLAM3.so, installs it with its headers, and exports ORB_SLAM3::ORB_SLAM3.
#
# Differences from upstream:
#   - Headless. Pangolin is gone from nixpkgs, and the rover has no display; Viewer.cc
#     and MapDrawer.cc are replaced with no-op versions (./headless), so a System must be
#     constructed with bUseViewer = false. The examples are not built.
#   - No -march=native: the package must run on whatever CPU it is deployed to.
#   - C++17 rather than C++11 (GCC 15 and Eigen 3.4); COMPILEDWITHC11 is still defined,
#     since the sources branch on it to pick std::chrono::steady_clock.
cmake_minimum_required(VERSION 3.18)
project(
  ORB_SLAM3
  VERSION 1.0.0
  LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost CONFIG REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)
find_package(Threads REQUIRED)

# g2o's generated config.h; no OpenMP, linked into the one library. It goes into the
# source tree because g2o includes it as "../../config.h", relative to its own sources.
set(G2O_OPENMP OFF)
set(G2O_SHARED_LIBS OFF)
configure_file(Thirdparty/g2o/config.h.in
               ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/g2o/config.h)

# The vocabulary ships compressed.
file(ARCHIVE_EXTRACT INPUT
     ${CMAKE_CURRENT_SOURCE_DIR}/Vocabulary/ORBvoc.txt.tar.gz DESTINATION
     ${CMAKE_CURRENT_BINARY_DIR}/vocabulary)

file(GLOB DBOW2_SOURCES Thirdparty/DBoW2/DBoW2/*.cpp
     Thirdparty/DBoW2/DUtils/*.cpp)
file(GLOB G2O_SOURCES Thirdparty/g2o/g2o/types/*.cpp
     Thirdparty/g2o/g2o/core/*.cpp Thirdparty/g2o/g2o/stuff/*.cpp
     Thirdparty/g2o/g2o/stuff/*.c)
file(GLOB ORB_SLAM3_SOURCES src/*.cc src/*.cpp src/CameraModels/*.cpp)

add_library(ORB_SLAM3 SHARED ${ORB_SLAM3_SOURCES} ${DBOW2_SOURCES}
                             ${G2O_SOURCES})
target_compile_definitions(
  ORB_SLAM3
  PUBLIC COMPILEDWITHC11
  PRIVATE UNIX)
# Third-party code, kept diffable against upstream: not held to any warning policy.
target_compile_options(ORB_SLAM3 PRIVATE -O3 -w)
target_include_directories(
  ORB_SLAM3
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/CameraModels>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/Sophus>
         $<INSTALL_INTERFACE:include/ORB_SLAM3>
         $<INSTALL_INTERFACE:include/ORB_SLAM3/include>
         $<INSTALL_INTERFACE:include/ORB_SLAM3/include/CameraModels>
         $<INSTALL_INTERFACE:include/ORB_SLAM3/Thirdparty/Sophus>
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/g2o/g2o/core
          ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/g2o/g2o/types
          ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/g2o/g2o/stuff)
target_link_libraries(
  ORB_SLAM3 PUBLIC ${OpenCV_LIBS} Eigen3::Eigen Boost::serialization
                   OpenSSL::Crypto Threads::Threads)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(
  TARGETS ORB_SLAM3
  EXPORT ORB_SLAM3Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
# Every header in its upstream place, since the public headers include each other and
# the third-party ones by paths relative to the repository root.
install(
  DIRECTORY include Thirdparty
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ORB_SLAM3
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/vocabulary/ORBvoc.txt
        DESTINATION ${CMAKE_INSTALL_DATADIR}/ORB_SLAM3)

install(
  EXPORT ORB_SLAM3Targets
  NAMESPACE ORB_SLAM3::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ORB_SLAM3)
file(
  WRITE ${CMAKE_CURRENT_BINARY_DIR}/ORB_SLAM3Config.cmake
  "include(CMakeFindDependencyMacro)
find_dependency(OpenCV 4)
find_dependency(Eigen3 3.3 NO_MODULE)
find_dependency(Boost CONFIG COMPONENTS serialization)
find_dependency(OpenSSL)
find_dependency(Threads)
include(\"\${CMAKE_CURRENT_LIST_DIR}/ORB_SLAM3Targets.cmake\")
# The ORB vocabulary, installed alongside the library.
get_filename_component(ORB_SLAM3_PREFIX \"\${CMAKE_CURRENT_LIST_DIR}/../../..\" ABSOLUTE)
set(ORB_SLAM3_VOCABULARY \"\${ORB_SLAM3_PREFIX}/${CMAKE_INSTALL_DATADIR}/ORB_SLAM3/ORBvoc.txt\")
")
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/ORB_SLAM3ConfigVersion.cmake
  COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ORB_SLAM3Config.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/ORB_SLAM3ConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ORB_SLAM3)
