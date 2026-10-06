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

  # Customize the the command prompt in the shell
  enterShell = ''
    # get the profile name
    dotfile="''${DEVENV_DOTFILE%/}"

    if [[ "$dotfile" == */profiles/* ]]; then
      profile_name="(''${dotfile##*/profiles/}) "
    else
      # no profile selected, no profile name to display
      profile_name=""
    fi

    # use https://bash-prompt-generator.org to change this
    PS1_CMD1=$(git branch --show-current 2>/dev/null); 
    PS1='\[\e[38;5;202;1m\]''${profile_name}\[\e[0m\][\[\e[38;5;81m\]\u\[\e[0m\]@\[\e[38;5;81m\]\h\[\e[0m\]:\[\e[38;5;81m\]\w\[\e[0m\]] \[\e[38;5;76;3m\]''${PS1_CMD1}\[\e[0m\] \\$ '
  '';
}
