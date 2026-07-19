# https://index.0x77.dev/blog/ros-devenv
{
  # Access to inputs from devenv.yaml
  pkgs,
  # lib,
  # config,
  # nixpkgs,
  nix-ros-overlay,
  nix-ros-workspace,
  # nixgl,
  ...
}:

let
  # isIntelX86Platform = pkgs.stdenv.system == "x86_64-linux";
  # nixGL = import nixgl {
  #   inherit pkgs;
  #   enable32bits = isIntelX86Platform;
  #   enableIntelX86Extensions = isIntelX86Platform;
  # };
  rosDistro = "jazzy";
  # packagesFromDirectoryRecursive returns a deep set and this converts to a list of derivations
  # flattenDerivationSet = set: (lib.collect lib.isDerivation set);
in
{
  name = "Perseus-v3";

  # Configure Cachix binary caches for faster builds
  cachix.pull = [ "ros" ]; # Pull pre-built ROS packages

  overlays = import ./nix/overlays.nix {
    inherit
      nix-ros-overlay
      nix-ros-workspace
      rosDistro
      ;
  };

  # --- Packages ---
  packages = with pkgs; [
    colcon # The ROS 2 build tool
    graphviz # Often needed for ROS visualization tools
    livox-sdk2
  ];
  # ++ flattenDerivationSet examples;
}
