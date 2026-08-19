{ pkgs, config, ... }:

{
  packages =
    with pkgs;
    (
      [
        libnice
        tsx
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
      npm.enable = true;
      corepack.enable = true;
    };
  };

  enterShell = "cd ${config.env.DEVENV_ROOT}/software/web_ui";

  tasks = {
    "web-ui:init" = {
      exec = ''
        cd ${config.env.DEVENV_ROOT}/software/web_ui
        # pipe yes to automatically install yarn if prompted
        yes | yarn
      '';
      after = [
        "devenv:enterShell"
        "devenv:enterTest"
      ];
    };
  };

  scripts = {
    "web-ui-dev" = {
      exec = ''
        cd ${config.env.DEVENV_ROOT}/software/web_ui
        vite dev --host
      '';
    };
    "web-ui-build" = {
      exec = ''
        cd ${config.env.DEVENV_ROOT}/software/web_ui
        vite build
        node ./src/server/server.js
      '';
    };
    "web-ui" = {
      exec = ''
        node ${config.env.DEVENV_ROOT}/software/web_ui/src/server/server.js
      '';
    };
    "camera-server" = {
      exec = ''
        tsx ${config.env.DEVENV_ROOT}/software/web_ui/pi-server/index.ts
      '';
    };
  };
}
