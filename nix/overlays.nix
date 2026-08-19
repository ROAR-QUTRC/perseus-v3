# These overlays are automatically applied to nixpkgs using the `overlays` attribute in devenv.nix
{
  rosDistro,
  nix-ros-overlay,
  nix-ros-workspace,
  ...
}:
[
  nix-ros-overlay.overlays.default # Makes ROS packages available via pkgs.rosPackages...
  # Add extra packages and patches
  (import ./extra-packages/overlay.nix rosDistro)
  # add ros workspace functionality
  nix-ros-workspace.overlays.default
  # import ros workspace packages + fixes
  (import ../software/overlay.nix)

  (final: prev: {
    # alias the output to pkgs.ros to make it easier to use
    ros = final.rosPackages.${rosDistro};
  })
]
