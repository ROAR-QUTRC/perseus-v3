{ pkgs, ... }:
{
  imports = [
    ./zsh.nix
  ];
  home.shell = pkgs.zsh;
}
