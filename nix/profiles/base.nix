{
  pkgs,
  lib,
  config,
  ...
}:
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
      # groot2
      # nixgl-script
      # nixcuda-script
      yaml-cpp
      graphviz # Often needed for ROS visualization tools
      # livox-sdk2
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

  # nix-ros-workspace wires buildROSWorkspace's `buildROSEnv` argument to
  # `rosFinal.buildEnv`, so overriding buildEnv in the ROS scope is what makes
  # the workspace it builds an underlay. Only the env builder changes, so no
  # package derivation is rebuilt.
  rosUnderlay = pkgs.ros.overrideScope (
    rosFinal: rosPrev: {
      buildEnv = args: rosPrev.buildEnv (args // { underlay = true; });
    }
  );

  # Build-time (dev) dependencies of the workspace source packages.
  # These must be dev packages (not prebuilt) so colcon gets their headers,
  # CMake config and link libraries. ROS dev packages also bring their
  # propagated dev closure (e.g. lttng-ust via rclcpp), so listing a direct
  # dependency is usually enough.
  # TODO: replace with per-package Nix derivations (ros2nix) once packaged.
  workspaceDevDeps = {
    inherit (pkgs)
      nlohmann_json # can_if, sensors
      openssl # sensors
      onnxruntime # vision (provides libonnxruntime.pc for pkg_check_modules)
      # opencv intentionally omitted: cv-bridge propagates the ROS-consistent
      # OpenCV; adding pkgs.opencv causes a buildEnv version conflict.
      ;
    inherit (pkgs.ros)
      # ros2_control / hardware
      hardware-interface # hardware, sensors
      pluginlib # hardware
      rclcpp-lifecycle # hardware, sensors
      # behaviour tree
      behaviortree-cpp # perseus_bt_nodes
      # messages / interfaces
      actuator-msgs # teleop
      nav-msgs # sensors (CMake-only, not in package.xml)
      rcl-interfaces # vision
      std-msgs # vision
      visualization-msgs # vision
      builtin-interfaces # interfaces, perseus_bt_nodes
      # rosidl codegen (interfaces)
      rosidl-default-generators # msg/srv codegen
      rosidl-default-runtime # generated interface runtime
      python-cmake-module # rosidl python bindings
      # tf / geometry
      tf2 # sensors, vision
      tf2-ros # vision
      tf2-geometry-msgs # sensors, vision
      # core / misc
      rclcpp # brings lttng-ust link libs (perseus_bt_nodes etc.)
      rclcpp-components # sensors (CMake-only, not in package.xml)
      backward-ros # hardware, sensors, teleop, interfaces
      ament-index-cpp # vision
      cv-bridge # vision
      # sensors
      realsense2-camera # sensors
      realsense2-description # sensors
      rplidar-ros # sensors
      # lint (test deps)
      ament-lint-auto # interfaces, perseus_bt_nodes, vision
      ament-lint-common # interfaces, perseus_bt_nodes, vision
      # navigation / localization
      robot-localization # autonomy
      slam-toolbox # autonomy
      navigation2 # autonomy
      xacro # autonomy
      ;
  };

  # Built against rosUnderlay, which flips the program wrappers from
  # `--prefix AMENT_PREFIX_PATH` to `--suffix`.
  #
  # Every workspace package is also packaged for Nix (nix/extra-packages) with
  # `src` pointing at the local source tree, so the workspace on PATH contains
  # a Nix-built copy of each one. With `--prefix`, the ros2 wrapper put those
  # copies ahead of anything the shell had exported, so a `colcon build` into
  # ./install could never win: edits to a URDF, launch file or param only took
  # effect after leaving and re-entering the shell, which rebuilt the
  # derivation from the changed source rather than using the colcon build.
  #
  # As an underlay the Nix copies still resolve when nothing else is sourced,
  # so running straight out of the shell is unchanged, but
  # `source install/setup.bash` now takes priority over them.
  defaultWorkspace = mkWorkspace {
    ros = rosUnderlay;
    name = "default";
    additionalDevPkgs = workspaceDevDeps;
    additionalPostShellHook = ''
      # onnxruntime ships libonnxruntime.pc in its dev output, which the
      # workspace buildEnv does not surface. Add it so vision's
      # pkg_check_modules(libonnxruntime) resolves. The .pc uses absolute
      # paths, so this also supplies the correct lib flags.
      export PKG_CONFIG_PATH="${pkgs.onnxruntime.dev}/lib/pkgconfig''${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
      # vision includes <onnxruntime/onnxruntime_cxx_api.h>, but the .pc
      # only advertises the .../include/onnxruntime subdir. Nix build inputs
      # normally get -isystem $dev/include automatically; replicate that here
      # so the "onnxruntime/"-prefixed include resolves.
      export CPATH="${pkgs.onnxruntime.dev}/include''${CPATH:+:$CPATH}"
    '';
  };

  rosWs = "${config.env.DEVENV_ROOT}/software/ros_ws";
in
{
  packages = [
    defaultWorkspace
  ];

  processes = {
    "perseus" = {
      exec = ''
        ${defaultWorkspace}/bin/ros2 launch perseus perseus.launch.py
      '';
    };
  };

  enterShell = ''
    # Pass the shell hook from the nix-ros-workspace shell to the devenv shell
    ${defaultWorkspace.env.shellHook}

    export COLCON_EXTENSION_BLOCKLIST=colcon_ros.prefix_path.ament

    # Pick up an existing colcon build so ros2 resolves the workspace packages
    # to ./install rather than the Nix-built copies. Safe when absent: the Nix
    # underlay still provides every package.
    if [ -f "${rosWs}/install/setup.bash" ]; then
      source "${rosWs}/install/setup.bash"
    fi

    # Rebuild the workspace and refresh the *current* shell, so source edits
    # apply without leaving and re-entering the profile. This has to be a
    # function rather than a devenv script: a script runs in a child process
    # and cannot source anything back into this shell.
    #
    # --symlink-install means launch files, YAML params and the teleop Python
    # scripts take effect on save with no rebuild at all; C++ still needs one.
    rebuild() {
      (cd "${rosWs}" && colcon build --symlink-install "$@") &&
        source "${rosWs}/install/setup.bash"
    }

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
