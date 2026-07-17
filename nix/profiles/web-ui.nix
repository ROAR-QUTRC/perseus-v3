{ pkgs, config, ... }:

{
  packages =
    with pkgs;
    (
      [
        libnice
      ]
      ++ (with gst_all_1; [
        gstreamer
        gst-plugins-base
        gst-plugins-good
        gst-plugins-bad
        gst-plugins-rs
      ])
    );

  languages = {
    typescript.enable = true;
    javascript = {
      enable = true;
      corepack.enable = true;
    };
  };

  enterShell = "cd ${config.env.DEVENV_ROOT}/software/web-ui";

  tasks = {
    "web-ui:init" = {
      exec = ''
        cd ${config.env.DEVENV_ROOT}/software/web-ui
        # pipe yes to automatically install yarn if prompted
        yes | yarn 
      '';
      after = [
        "devenv:enterShell"
        "devenv:enterTest"
      ];
    };
  };
}
