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
# Twist gets its own pair -- publish.odom_linear_velocity_variance / _angular_ ((m/s)^2 /
# (rad/s)^2) -- rather than reusing the pose pair above, which would have been dimensionally
# wrong. Worth having: publishLatestState fills a real linear_velocity from the optimizer's
# own state, unlike FAST-LIO, which never populates its odometry's twist at all. (Fusing it
# into the Lunabotics EKF was tried anyway and made it measurably noisier -- see
# ekf_config.yaml -- so this pair is currently unused there, not a recommendation to fuse it.)
#
# Pose covariance is no longer just that flat pair, either: LsqRegistration derives a real
# per-scan estimate from the registration's own Gauss-Newton Hessian
# (poseCovarianceDiagonal), so an ill-constrained direction (yaw and the horizontal plane on
# a flat sand floor, the common case here) is reported as uncertain instead of getting the
# same fixed number as a well-constrained one. odom_position_variance /
# odom_orientation_variance are now that estimate's fallback, used only when it is not
# trustworthy this scan (too few effective points, or too ill-conditioned).
#
# All five covariance keys (publish.enable_odom_covariance and the four variances) are also
# ordinary ROS2 parameters on the bievr_lio node -- `ros2 param set /bievr_lio <name> <value>`
# changes them live, not only this file at startup.
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
  rev = "fa6f8b8a18e1d277eb080ff668f4332c6f41b518"; # feat/stale-pixel-decay
  hash = "sha256-fWKWBvNFjdJhGhBw4T8jSjQ00zeN8unzFUiuxWsasC0=";
}
