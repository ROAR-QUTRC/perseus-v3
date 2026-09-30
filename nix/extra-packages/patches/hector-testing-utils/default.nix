{
  lib,
  buildRosPackage,
  fetchFromGitHub,
  ament-cmake,
  rcl-interfaces,
  rclcpp,
  rclcpp-action,
  std-msgs,
}:
buildRosPackage rec {
  pname = "ros-jazzy-hector-testing-utils";
  version = "0.0.0";

  # Not released in the ROS Jazzy distro, so there is no nix-ros-overlay package
  # for it. Matches the vcstool entry in
  # software/ros_ws/src/hector_transmission_interface/dependencies.repos.
  # Pinned to a commit rather than the moving main branch. To move the pin:
  #   git ls-remote https://github.com/Joschi3/hector_testing_utils.git main
  #   nix-prefetch-url --unpack https://github.com/Joschi3/hector_testing_utils/archive/<rev>.tar.gz
  src = fetchFromGitHub {
    owner = "Joschi3";
    repo = "hector_testing_utils";
    rev = "4eb7f763b19420c407fa20bef960c563dc060e6e"; # main
    hash = "sha256-3PeJq6xlRz+uADgkqnByRgWwgnLJeBtZ4wBh52Ljaqg=";
  };

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake ];
  propagatedBuildInputs = [
    rcl-interfaces
    rclcpp
    rclcpp-action
    std-msgs
  ];
  nativeBuildInputs = [ ament-cmake ];

  meta = {
    description = "Helper classes and functions for writing tests in ros2.";
    license = with lib.licenses; [ bsd3 ];
  };
}
