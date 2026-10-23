# https://index.0x77.dev/blog/ros-devenv
{
  # Access to inputs from devenv.yaml
  pkgs,
  config,
  # nixpkgs,
  nix-ros-overlay,
  nix-ros-workspace,
  local-date,
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
in
{
  name = "Perseus-v3";

  overlays = import ./nix/overlays.nix {
    inherit
      nix-ros-overlay
      nix-ros-workspace
      local-date
      rosDistro
      ;
  };

  # only include scripts and examples by default
  packages = with pkgs; [ ] ++ flattenDerivationSet examples ++ flattenDerivationSet scripts;
  # alias to open the VSCode workspace
  scripts.perseus-workspace.exec = "code ${config.env.DEVENV_ROOT}/perseus-v3.code-workspace";

  # Only used for debugging: Can do `devenv build outputs.pkgs.XXX` to build a specific package
  outputs = {
    # Needs to be a derivation because devenv doesn't like pkgs.lib.recurseIntoAttrs for some reason
    pkgs = pkgs.runCommand "roar-all-pkgs" { passthru = pkgs; } "touch $out";
  };
}
