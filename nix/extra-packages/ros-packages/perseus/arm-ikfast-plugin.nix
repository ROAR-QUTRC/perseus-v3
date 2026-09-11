{
  lib,
  buildRosPackage,
  ament-cmake,
  blas,
  generate-parameter-library,
  lapack,
  moveit-core,
  orocos-kdl-vendor,
  pluginlib,
  rclcpp,
  tf2-eigen,
  tf2-kdl,
}:
buildRosPackage rec {
  pname = "ros-jazzy-arm-ikfast-plugin";
  version = "0.0.1";

  src = ./../../../../software/ros_ws/src/arm_ikfast_plugin;

  buildType = "ament_cmake";
  buildInputs = [
    ament-cmake
    blas
    lapack
  ];
  propagatedBuildInputs = [
    generate-parameter-library
    moveit-core
    orocos-kdl-vendor
    pluginlib
    rclcpp
    tf2-eigen
    tf2-kdl
  ];
  nativeBuildInputs = [ ament-cmake ];

  meta = {
    description = "Generated IKFast kinematics plugin for the three-joint Perseus arm group.";
    license = with lib.licenses; [ mit ];
  };
}
