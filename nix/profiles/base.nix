{ pkgs, ... }:
let
  productionDomainId = 42;
  devDomainId = 51;

  # --- INPUT PACKAGE SETS ---
  devPackages = pkgs.ros.devPackages // pkgs.sharedDevPackages // pkgs.nativeDevPackages;
  # Packages which should be base profile
  standardPkgs = {
    inherit (pkgs)
      bashInteractive
      can-utils
      glibcLocales
      # groot2
      ncurses
      # nixgl-script
      # nixcuda-script
      yaml-cpp
      ;
    inherit (pkgs.ros)
      ros-core
      ament-cmake-core

      demo-nodes-cpp
      joy
      # livox-ros-driver2
      opennav-docking
      nav2-common
      nav2-lifecycle-manager
      nav2-msgs
      nav2-rviz-plugins
      nav2-util
      rmw-cyclonedds-cpp
      rosbag2
      rosbridge-suite
      rqt-reconfigure
      rqt-plot
      rqt-common-plugins
      rqt-graph
      rqt-gui
      rqt-gui-py
      # rviz2-fixed
      teleop-twist-keyboard
      tf2-tools
      twist-stamper
      ;
  };
  # Packages which should be available only in the dev shell
  devShellPkgs = {
    inherit (pkgs)
      man-pages
      man-pages-posix
      stdmanpages
      # nix-gl-host
      ;
  };

  # --- ROS WORKSPACES ---
  # function to build a ROS workspace which modifies the dev shell hook to set up environment variables
  mkWorkspace =
    {
      ros,
      name ? "ROAR",
      additionalDevPkgs ? { },
      additionalPkgs ? { },
      additionalPrebuiltPkgs ? { },
      additionalPostShellHook ? "",
    }:
    ros.callPackage ros.buildROSWorkspace {
      inherit name;
      devPackages = devPackages // additionalDevPkgs;
      prebuiltPackages = standardPkgs // additionalPkgs;
      prebuiltShellPackages = devShellPkgs // additionalPrebuiltPkgs;
      releaseDomainId = productionDomainId;
      environmentDomainId = devDomainId;
      forceReleaseDomainId = true;

      postShellHook = ''
        # use CycloneDDS ROS middleware
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        # enable coloured ros2 launch output
        export RCUTILS_COLORIZED_OUTPUT=1
        # fix locale issues
        export LOCALE_ARCHIVE=${pkgs.glibcLocales}/lib/locale/locale-archive
      ''
      + additionalPostShellHook;
    };

  defaultWorkspace = mkWorkspace {
    inherit (pkgs) ros;
    name = "default";
  };
in
{
  packages = [
    defaultWorkspace
  ];

  enterShell = ''
    # Pass the shell hook from the nix-ros-workspace shell to the devenv shell
    ${defaultWorkspace.env.shellHook}

    export COLCON_EXTENSION_BLOCKLIST=colcon_ros.prefix_path.ament

    echo -e "\e[38;5;208m______                                    _____ ";
    echo -e "| ___ \\                                  |____ |";
    echo -e "| |_/ /__ _ __ ___  ___ _   _ ___  __   __   / /";
    echo -e "|  __/ _ \\ '__/ __|/ _ \\ | | / __| \\ \\ / /   \\ \\";
    echo -e "| | |  __/ |  \\__ \\  __/ |_| \\__ \\  \\ V /.___/ /";
    echo -e "\\_|  \\___|_|  |___/\\___|\\__,_|___/   \\_/ \\____/ ";
    # echo -e "------------------------------------------------";
    echo -e "QUTRC - Remote Off-world Autonomous Robotics\e[0m";
  '';
}
