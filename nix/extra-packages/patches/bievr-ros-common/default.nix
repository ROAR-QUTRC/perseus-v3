# Header-only glue shared by the ROS1 and ROS2 wrappers: the YAML config loader and the
# templated message conversions. Carries no ROS dependency of its own -- the message types
# are supplied by whichever wrapper includes the headers.
{
  lib,
  buildRosPackage,
  callPackage,
  fetchFromGitHub,
  cmake,
  bievr-lio,
}:
buildRosPackage {
  pname = "ros-jazzy-bievr-ros-common";
  version = "0.0.0";

  src = callPackage ../bievr-lio/src.nix { inherit fetchFromGitHub; };
  sourceRoot = "source/interfaces/ros_common";

  buildType = "cmake";
  nativeBuildInputs = [ cmake ];
  propagatedBuildInputs = [ bievr-lio ];

  meta = {
    description = "ROS-agnostic glue shared by the BIEVR-LIO ROS1 and ROS2 wrappers";
    license = with lib.licenses; [ bsd3 ];
  };
}
