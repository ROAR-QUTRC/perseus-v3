# The ROS2 wrapper: the process_topics / process_bag executables, the launch files and the
# canonical YAML configs.
{
  lib,
  buildRosPackage,
  callPackage,
  fetchFromGitHub,
  ament-cmake,
  bievr-lio,
  bievr-ros-common,
  geometry-msgs,
  livox-ros-driver2,
  nav-msgs,
  rclcpp,
  rosbag2-cpp,
  rosbag2-storage,
  sensor-msgs,
  std-msgs,
  std-srvs,
  tbb,
  tf2,
  tf2-ros,
  yaml-cpp,
}:
buildRosPackage {
  pname = "ros-jazzy-bievr-lio-ros2";
  version = "0.0.0";

  src = callPackage ../bievr-lio/src.nix { inherit fetchFromGitHub; };
  # Builds from interfaces/ros2, but not in isolation: its CMakeLists installs
  # ../../config (the shared YAMLs at the repository root) into this package's share, so
  # the whole tree has to be unpacked and only the working directory moves.
  sourceRoot = "source/interfaces/ros2";

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake ];
  nativeBuildInputs = [ ament-cmake ];
  propagatedBuildInputs = [
    bievr-lio
    bievr-ros-common
    geometry-msgs
    # Optional upstream (find_package(... QUIET)), and the reason to include it: with it
    # present the wrapper compiles the CustomMsg branch, so a Livox driver publishing its
    # native message is handled as well as one publishing PointCloud2.
    livox-ros-driver2
    nav-msgs
    rclcpp
    rosbag2-cpp
    rosbag2-storage
    sensor-msgs
    std-msgs
    # /map_save is a std_srvs/Trigger
    std-srvs
    tbb
    tf2
    # the wrapper reads lidar_frame -> base_frame from TF and broadcasts the optional
    # static imu -> lidar transform
    tf2-ros
    # config_loader.h parses the plain YAML config files; the package.xml asks for
    # yaml_cpp_vendor, but find_package(yaml-cpp) is what CMakeLists.txt actually does
    yaml-cpp
  ];

  meta = {
    description = "ROS2 (ament_cmake) interface for the BIEVR-LIO library";
    license = with lib.licenses; [ bsd3 ];
  };
}
