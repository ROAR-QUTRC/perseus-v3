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
# Also fixes /Odometry's pose and twist covariance, which upstream leaves at zero on every
# message -- nothing in the pipeline or either ROS wrapper ever wrote to them. A zero
# covariance tells a downstream consumer (a robot_localization EKF, here) that the
# measurement is exact, defeating both its blending against its own prior and any
# Mahalanobis gating it does: confirmed on a Lunabotics sand-arena bag as a ~3.7 m EKF
# teleport-and-back over three corrections whose raw /Odometry input was smooth throughout.
# publish.odom_position_variance / publish.odom_orientation_variance (diagonal-only, m^2 /
# rad^2) control it; both default to a placeholder sized to that bag's measured jitter.
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
  rev = "3527e9f8aaae1b71191ec0d3109ebf5cdef6306a"; # feat/stale-pixel-decay
  hash = "sha256-OYsz5NpvAAinVQ6LEYfTH0EF6gZ6nUbTldjnPf0IlKU=";
}
