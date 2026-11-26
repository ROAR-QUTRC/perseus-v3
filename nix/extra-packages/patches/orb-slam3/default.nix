# ORB-SLAM3 as a headless, installable library: plain CMake, no ROS. vision's
# orb_slam_odometry component finds it with find_package(ORB_SLAM3) and gets the ORB
# vocabulary from ORB_SLAM3_VOCABULARY (share/ORB_SLAM3/ORBvoc.txt).
#
# Upstream (UZ-SLAMLab/ORB_SLAM3, 2021) builds in-tree with build.sh and installs nothing,
# needs Pangolin (removed from nixpkgs) for its viewer, and no longer compiles on this
# toolchain (GCC 15, C++17). So:
#   CMakeLists.txt      replaces upstream's: one shared library with DBoW2 and g2o built
#                       in, installed headers and an exported ORB_SLAM3::ORB_SLAM3 target
#   headless/           Viewer and MapDrawer without Pangolin; same interface, no-ops
#   headless.patch      drops the remaining Pangolin includes and the GL type in Map
#   global-bundle-adjustment-counter.patch
#                       LoopClosing counts global bundle adjustments in a bool, which C++17
#                       refuses to increment -- and which never counted past 1 anyway, so
#                       a newer adjustment could not tell an older one to abort; now an int
#   rectified-stereo-settings-print.patch
#                       printing the settings dereferenced the second camera's model, which
#                       a "Rectified" stereo setup never creates: a segfault on startup
#
# GPL-3.0: linked only into vision's orb_slam_odometry component, kept apart from the
# MIT-licensed detector components the same way libviso2 was.
{
  lib,
  stdenv,
  fetchFromGitHub,
  cmake,
  boost,
  eigen,
  opencv,
  openssl,
}:
stdenv.mkDerivation {
  pname = "orb-slam3";
  version = "1.0-unstable-2022-12-22";

  # Pinned to a commit rather than a branch, since fetchFromGitHub does not track a moving
  # ref. To move it: nix-prefetch-url --unpack <archive url>, then nix hash convert.
  src = fetchFromGitHub {
    owner = "UZ-SLAMLab";
    repo = "ORB_SLAM3";
    rev = "4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4";
    hash = "sha256-fqG0g2zo2xPhUoU79ahU3OFRDAyYzldUp9osB5uCo/U=";
  };

  patches = [
    ./headless.patch
    ./global-bundle-adjustment-counter.patch
    ./rectified-stereo-settings-print.patch
  ];

  postPatch = ''
    cp ${./CMakeLists.txt} CMakeLists.txt
    cp ${./headless/MapDrawer.h} include/MapDrawer.h
    cp ${./headless/MapDrawer.cc} src/MapDrawer.cc
    cp ${./headless/Viewer.cc} src/Viewer.cc
  '';

  nativeBuildInputs = [ cmake ];
  propagatedBuildInputs = [
    boost
    eigen
    opencv
    openssl
  ];

  meta = {
    description = "ORB-SLAM3 visual(-inertial) SLAM library, headless";
    homepage = "https://github.com/UZ-SLAMLab/ORB_SLAM3";
    license = lib.licenses.gpl3Plus;
    platforms = lib.platforms.linux;
  };
}
