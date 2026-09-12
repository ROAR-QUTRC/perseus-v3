# The pinned source of the BIEVR-LIO fork, shared by the three packages built out of it
# (bievr-lio, bievr-ros-common, bievr-lio-ros2). One file so the three can never drift onto
# different commits of the same repository.
#
# Fork of ethz-asl/BIEVR-LIO. It carries the TF frame names, TF-publishing switches and
# base-relative pose options turned into config keys (lidar.frame, base.frame,
# base.odom_in_base / origin_at_base / heading_at_base, publish.tf / tf_lidar), plus
# lidar.allow_untimed for simulated clouds, calibration.from_tf so the LiDAR-IMU extrinsic
# can come out of the robot description, configurable subscription QoS and bounded
# synchronizer queues, and map publishing (publish.map_interval_s) plus a map_save
# service, neither of which upstream has at all. Also carries map.stale_timeout_s: a bump-
# image pixel not reconfirmed within that many seconds is cleared instead of averaged into
# forever, so terrain that genuinely changes (e.g. fine sand reshaped by wheels) can
# overwrite the old map instead of being diluted by it. 0 (the default) disables it. A
# stale pixel is only actually cleared if this scan's point count in the voxel is at least
# map.stale_min_relative_density (default 0.5) times its currently-observed pixel count, so
# a long-range or grazing-angle look -- naturally sparser than what built the surface -- is
# not mistaken for the surface being gone.
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
  rev = "c1f9b8dd41ed536f86d1eaf9775331da8979102b"; # feat/stale-pixel-decay
  hash = "sha256-f1LHAwxPrPdFBwtvYGtAv3TrBcv+ZDvFZ4rtQuV1Uw4=";
}
