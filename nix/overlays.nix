# These overlays are automatically applied to nixpkgs using the `overlays` attribute in devenv.nix
{
  rosDistro,
  nix-ros-overlay,
  nix-ros-workspace,
  local-date,
  ...
}:
[
  nix-ros-overlay.overlays.default # Makes ROS packages available via pkgs.rosPackages...
  # fix colcon (silence warnings, add extensions)
  (import ./extra-packages/colcon/overlay.nix)
  # base station time server (pkgs.local-date)
  (import ./extra-packages/local-date/overlay.nix local-date)
  # behavior tree viewer (pkgs.groot2)
  (import ./extra-packages/groot2/overlay.nix)
  # add ros workspace functionality
  nix-ros-workspace.overlays.default
  # import ros workspace packages + fixes
  (import ../software/overlay.nix rosDistro)

  (final: prev: {
    # alias the output to pkgs.ros to make it easier to use
    ros = final.rosPackages.${rosDistro};
  })
]
