{ pkgs, lib, ... }:
let
  productionDomainId = 42;
  devDomainId = 51;

  # --- INPUT PACKAGE SETS ---
  devPackages = pkgs.ros.devPackages // pkgs.sharedDevPackages // pkgs.nativeDevPackages;
  # Packages which should be base profile
  standardPkgs = {
    inherit (pkgs)
      colcon
      bashInteractive
      can-utils
      glibcLocales
      # nixgl-script
      # nixcuda-script
      yaml-cpp
      graphviz # Often needed for ROS visualization tools
      ;
    inherit (pkgs.ros)
      ros-core
      ament-cmake-core
      python-cmake-module

      demo-nodes-cpp
      joy
      livox-ros-driver2
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

    # Put the workspace's *dev* environment ahead of the workspace itself on PATH.
    #
    # buildROSWorkspace produces two environments: the workspace, which holds prebuilt
    # copies of every package including the ones being developed, and the dev
    # environment, which excludes those and carries only their dependencies. The hook
    # above already puts the dev environment first on AMENT_PREFIX_PATH, but PATH is left
    # alone, so ros2, python3 and xacro resolve to the workspace's copies -- and those are
    # wrapped to strip and re-prepend the workspace onto AMENT_PREFIX_PATH:
    #
    #   AMENT_PREFIX_PATH='/nix/store/...-workspace'$AMENT_PREFIX_PATH
    #
    # which undoes the hook from inside every process. The effect is that a package built
    # locally with colcon can never win over its prebuilt copy: xacro's $(find pkg),
    # pluginlib and CMake's find_package all silently resolve to the prebuilt one.
    #
    # Taking the binaries from the dev environment instead means their wrappers prepend
    # an environment that does not contain the workspace's own packages, so the colcon
    # install/ tree takes priority. The workspace stays on PATH behind it, since it
    # supplies mk-workspace-shell-setup and the non-ROS closure the dev environment omits.
    export PATH="$ROS_WORKSPACE_ENV_PATH/bin:$PATH"

    printf '\e[38;5;208m'
    echo "______                                    _____ ";
    echo "| ___ \\                                  |____ |";
    echo "| |_/ /__ _ __ ___  ___ _   _ ___  __   __   / /";
    echo "|  __/ _ \\ '__/ __|/ _ \\ | | / __| \\ \\ / /   \\ \\";
    echo "| | |  __/ |  \\__ \\  __/ |_| \\__ \\  \\ V /.___/ /";
    echo "\\_|  \\___|_|  |___/\\___|\\__,_|___/   \\_/ \\____/ ";
    echo "QUTRC - Remote Off-world Autonomous Robotics";
    printf '\e[0m'
  '';
}
