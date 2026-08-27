# The core LIO library: plain CMake, no ROS at all. The two ROS wrappers find it through
# the bievr_lioConfig.cmake it installs.
{
  lib,
  buildRosPackage,
  callPackage,
  fetchFromGitHub,
  cmake,
  ceres-solver,
  eigen,
  glog,
  tbb,
}:
buildRosPackage {
  pname = "ros-jazzy-bievr-lio";
  version = "0.0.0";

  src = callPackage ./src.nix { inherit fetchFromGitHub; };
  # The repository holds three packages; this one lives in BIEVR/.
  sourceRoot = "source/BIEVR";

  buildType = "cmake";
  nativeBuildInputs = [ cmake ];
  propagatedBuildInputs = [
    # Ceres re-exports Eigen and glog, but all three are found by name in CMakeLists.txt,
    # so all three have to be here for find_package to resolve them.
    ceres-solver
    eigen
    glog
    # oneTBB: the code uses tbb::global_control and tbb::this_task_arena, neither of which
    # exists in the 2020 series.
    tbb
  ];

  meta = {
    description = "BIEVR-LIO: LiDAR-inertial odometry against a voxel-image map";
    license = with lib.licenses; [ bsd3 ];
  };
}
