{ pkgs, ... }:
{
  home.packages = with pkgs; [
    devenv
  ];
  programs.zsh.initExtra = ''
    eval "$(devenv hook zsh)"
  '';
}
