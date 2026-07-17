{ pkgs, config, ... }:
{
  profiles = {
    # Waiting for docs release but autoactivation should drop you into this shell
    default.module = {

      enterShell = ''
        echo -e "\e[38;5;208m______                                    _____ ";
        echo -e "| ___ \\                                  |____ |";
        echo -e "| |_/ /__ _ __ ___  ___ _   _ ___  __   __   / /";
        echo -e "|  __/ _ \\ '__/ __|/ _ \\ | | / __| \\ \\ / /   \\ \\";
        echo -e "| | |  __/ |  \\__ \\  __/ |_| \\__ \\  \\ V /.___/ /";
        echo -e "\\_|  \\___|_|  |___/\\___|\\__,_|___/   \\_/ \\____/ ";
        # echo -e "------------------------------------------------";
        echo -e "QUTRC - Remote Off-world Autonomous Robotics\e[0m";
      '';
    };

    web-ui.module = {
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
    };

    simulation = {
      extends = [ "default" ];
      module = {

      };
    };
  };
}
