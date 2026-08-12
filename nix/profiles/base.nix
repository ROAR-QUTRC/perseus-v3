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

  # Build-time (dev) dependencies of the workspace source packages.
  # These must be dev packages (not prebuilt) so colcon gets their headers,
  # CMake config and link libraries. ROS dev packages also bring their
  # propagated dev closure (e.g. lttng-ust via rclcpp), so listing a direct
  # dependency is usually enough.
  # TODO: replace with per-package Nix derivations (ros2nix) once packaged.
  workspaceDevDeps = {
    inherit (pkgs)
      nlohmann_json # perseus_can_if, perseus_sensors
      openssl # perseus_sensors
      onnxruntime # perseus_vision (provides libonnxruntime.pc for pkg_check_modules)
      # opencv intentionally omitted: cv-bridge propagates the ROS-consistent
      # OpenCV; adding pkgs.opencv causes a buildEnv version conflict.
      ;
    inherit (pkgs.ros)
      # ros2_control / hardware
      hardware-interface # perseus_hardware, perseus_sensors
      pluginlib # perseus_hardware
      rclcpp-lifecycle # perseus_hardware, perseus_sensors
      # behaviour tree
      behaviortree-cpp # perseus_bt_nodes
      # messages / interfaces
      actuator-msgs # perseus_teleop
      nav-msgs # perseus_sensors (CMake-only, not in package.xml)
      rcl-interfaces # perseus_vision
      std-msgs # perseus_vision
      visualization-msgs # perseus_vision
      builtin-interfaces # perseus_interfaces, perseus_bt_nodes
      # rosidl codegen (perseus_interfaces)
      rosidl-default-generators # msg/srv codegen
      rosidl-default-runtime # generated interface runtime
      python-cmake-module # rosidl python bindings
      # tf / geometry
      tf2 # perseus_sensors, perseus_vision
      tf2-ros # perseus_vision
      tf2-geometry-msgs # perseus_sensors, perseus_vision
      # core / misc
      rclcpp # brings lttng-ust link libs (perseus_bt_nodes etc.)
      rclcpp-components # perseus_sensors (CMake-only, not in package.xml)
      backward-ros # perseus_hardware, perseus_sensors, perseus_teleop, perseus_interfaces
      ament-index-cpp # perseus_vision
      cv-bridge # perseus_vision
      # sensors
      realsense2-camera # perseus_sensors
      realsense2-description # perseus_sensors
      rplidar-ros # perseus_sensors
      # lint (test deps)
      ament-lint-auto # perseus_interfaces, perseus_bt_nodes, perseus_vision
      ament-lint-common # perseus_interfaces, perseus_bt_nodes, perseus_vision
      # navigation / localization
      robot-localization # autonomy
      slam-toolbox # autonomy
      navigation2 # autonomy
      xacro # autonomy
      ;
  };

  defaultWorkspace = mkWorkspace {
    inherit (pkgs) ros;
    name = "default";
    additionalDevPkgs = workspaceDevDeps;
    additionalPostShellHook = ''
      # onnxruntime ships libonnxruntime.pc in its dev output, which the
      # workspace buildEnv does not surface. Add it so perseus_vision's
      # pkg_check_modules(libonnxruntime) resolves. The .pc uses absolute
      # paths, so this also supplies the correct lib flags.
      export PKG_CONFIG_PATH="${pkgs.onnxruntime.dev}/lib/pkgconfig''${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
      # perseus_vision includes <onnxruntime/onnxruntime_cxx_api.h>, but the .pc
      # only advertises the .../include/onnxruntime subdir. Nix build inputs
      # normally get -isystem $dev/include automatically; replicate that here
      # so the "onnxruntime/"-prefixed include resolves.
      export CPATH="${pkgs.onnxruntime.dev}/include''${CPATH:+:$CPATH}"
    '';
  };
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
