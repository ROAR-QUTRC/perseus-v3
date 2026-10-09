# https://index.0x77.dev/blog/ros-devenv
{
  # Access to inputs from devenv.yaml
  pkgs,
  lib,
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
  flattenDerivationSet = set: (lib.collect lib.isDerivation set);
in
{
  name = "Perseus-v3";

  # Port used by rosbridge_websocket. perseus.launch.py and the `rosbridge` script below
  # read ROSBRIDGE_PORT; the web UI reads PUBLIC_ROSBRIDGE_PORT (see
  # software/web_ui/src/lib/scripts/rosBridge.svelte.ts). Keep both in sync.
  env.ROSBRIDGE_PORT = "9090";
  env.PUBLIC_ROSBRIDGE_PORT = "9090";

  scripts = {
    rosbridge = {
      exec = "ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=$ROSBRIDGE_PORT";
      description = "Start rosbridge_websocket on $ROSBRIDGE_PORT";
    };
  };

  overlays = import ./nix/overlays.nix {
    inherit
      nix-ros-overlay
      nix-ros-workspace
      rosDistro
      ;
  };

  # --- Packages ---
  packages = with pkgs; [ ] ++ flattenDerivationSet examples ++ flattenDerivationSet scripts;

  # Only used for debugging: Can do `devenv build outputs.pkgs.XXX` to build a specific package
  outputs = {
    # Needs to be a derivation because devenv doesn't like pkgs.lib.recurseIntoAttrs for some reason
    pkgs = pkgs.runCommand "roar-all-pkgs" { passthru = pkgs; } "touch $out";
  };
}
