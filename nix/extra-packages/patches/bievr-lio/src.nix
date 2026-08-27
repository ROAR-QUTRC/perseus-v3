# The pinned source of the BIEVR-LIO fork, shared by the three packages built out of it
# (bievr-lio, bievr-ros-common, bievr-lio-ros2). One file so the three can never drift onto
# different commits of the same repository.
#
# Fork of ethz-asl/BIEVR-LIO. It carries the TF frame names, TF-publishing switches and
# base-relative pose options turned into config keys (lidar.frame, base.frame,
# base.odom_in_base / origin_at_base / heading_at_base, publish.tf / tf_lidar), plus
# lidar.allow_untimed for simulated clouds, calibration.from_tf so the LiDAR-IMU extrinsic
# can come out of the robot description, configurable subscription QoS and bounded
# synchronizer queues.
#
# Pinned to a commit rather than a branch name, since fetchFromGitHub does not track a
# moving ref and a bare branch would silently change what gets built.
#
# To move the pin, push the fork and re-read both values from:
#   nix-prefetch-git --url https://github.com/bocho0600/BIEVR-LIO --rev <commit>
{ fetchFromGitHub }:
fetchFromGitHub {
  owner = "bocho0600";
  repo = "BIEVR-LIO";
  rev = "0cd2b053fb5f5418ecc6d9414f9cf206ff5993c3"; # feat/configurable-frames
  hash = "sha256-gGS4L0NGrEG+rhawPlcS+xsWPzYwGjDQaamjFBFx7eo=";
}
