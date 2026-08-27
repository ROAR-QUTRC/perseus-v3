# ImMesh's incremental mesher, wrapped as a ROS 2 node and built without the LiDAR-inertial
# odometry it normally ships with. Runs at the base station: it reads the decompressed map
# cloud coming off the Draco link and publishes the reconstructed surface as a marker, so
# the mesh is built where there is CPU to spare and never crosses the radio link.
#
# Fork of hku-mars/ImMesh. It carries the IMMESH_STANDALONE split (the mesher without the
# odometry), a pose-aware entry point, and interfaces/ros2 -- none of which upstream has.
#
# Pinned to a commit rather than a branch: fetchFromGitHub does not track a moving ref.
# To move the pin, push the fork and re-read both values from:
#   nix-prefetch-git --url https://github.com/bocho0600/ImMesh --rev <commit>
{
  lib,
  buildRosPackage,
  fetchFromGitHub,
  ament-cmake,
  cgal_5,
  eigen,
  geometry-msgs,
  gmp,
  mpfr,
  nav-msgs,
  opencv,
  pcl,
  pcl-conversions,
  rclcpp,
  sensor-msgs,
  std-srvs,
  visualization-msgs,
}:
buildRosPackage {
  pname = "ros-jazzy-immesh-ros2";
  version = "0.0.0";

  src = fetchFromGitHub {
    owner = "bocho0600";
    repo = "ImMesh";
    rev = "e5c609a2bea0ef2b84eb1e7f717495e2c5a426c0"; # feat/ros2-meshing
    hash = "sha256-BglFz5HhraXoU0/e/nr0fO2SbFLRqGK9Jplpf9xfk9U=";
  };
  # The package lives in interfaces/ros2 but compiles sources from the repository root, so
  # the whole tree is unpacked and only the working directory moves.
  sourceRoot = "source/interfaces/ros2";

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake ];
  nativeBuildInputs = [ ament-cmake ];
  propagatedBuildInputs = [
    # CGAL 5, not 6, and deliberately: CGAL 6 returns std::variant from intersection()
    # where 5 returns boost::variant, and tools_graphics.hpp reads it with boost::get.
    # Pinning the version the code was written against beats rewriting the geometry that
    # decides triangle topology.
    cgal_5
    eigen
    geometry-msgs
    # CGAL's own CMake looks for both, and does not find them through cgal_5 alone.
    gmp
    mpfr
    nav-msgs
    opencv
    pcl
    pcl-conversions
    rclcpp
    sensor-msgs
    std-srvs
    visualization-msgs
  ];

  meta = {
    description = "ImMesh's incremental mesher as a ROS 2 node, without its LiDAR odometry";
    license = with lib.licenses; [ gpl2Only ];
  };
}
