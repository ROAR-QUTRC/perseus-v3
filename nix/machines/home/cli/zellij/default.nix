{
  programs.zellij = {
    enable = true;
    enableZshIntegration = true;
    extraConfig = ''
      simplified_ui true
    '';
  };
}
