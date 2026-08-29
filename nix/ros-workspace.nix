{ pkgs, ... }:
rec {
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
      # nixgl-script
      # nixcuda-script
      yaml-cpp
      graphviz # Often needed for ROS visualization tools
      # livox-sdk2
      ;
    inherit (pkgs.ros)
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
      isDev ? false,
      name ? "ROAR",
      additionalDevPkgs ? { },
      additionalPkgs ? { },
      additionalPrebuiltPkgs ? { },
      additionalPostShellHook ? "",
    }:
    ros.callPackage ros.buildROSWorkspace {
      inherit name;
      # in dev we dont want pkgs we maintain to be built by nix (so we can develop them)
      devPackages = if isDev then additionalDevPkgs else devPackages // additionalDevPkgs;
      prebuiltPackages = standardPkgs // additionalPkgs;
      prebuiltShellPackages =
        if isDev then devShellPkgs // additionalPrebuiltPkgs else additionalPrebuiltPkgs;
      releaseDomainId = productionDomainId;
      environmentDomainId = devDomainId;
      forceReleaseDomainId = true;

      postShellHook = ''
        # enable coloured ros2 launch output
        export RCUTILS_COLORIZED_OUTPUT=1
        # fix locale issues
        export LOCALE_ARCHIVE=${pkgs.glibcLocales}/lib/locale/locale-archive
      ''
      + additionalPostShellHook;
    };
}
