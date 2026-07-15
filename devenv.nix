# https://index.0x77.dev/blog/ros-devenv
{
  pkgs, # The Nix package set (derived from inputs.nixpkgs)
  lib, # Nixpkgs utility functions
  config, # devenv configuration values
  nixpkgs, # Direct access to the nixpkgs input
  nix-ros-overlay, # Direct access to the overlay input
  nixgl, # Direct access to the nixgl input
  ...
}:

let
  # Import custom package definitions (see vendor section below)
  # vendor = import ./vendor;
  # Helper for nixGL configuration
  isIntelX86Platform = pkgs.stdenv.system == "x86_64-linux";
  nixGL = import nixgl {
    inherit pkgs;
    enable32bits = isIntelX86Platform;
    enableIntelX86Extensions = isIntelX86Platform;
  };
in
{
  # A name for your environment
  name = "Perseus-v3"; # Replace with your project name

  # Configure Cachix binary caches for faster builds
  cachix.pull = [ "ros" ]; # Pull pre-built ROS packages
  cachix.push = "rover-test"; # Optional: Push your builds to a private cache

  # Apply overlays to the base package set
  overlays = [
    nix-ros-overlay.overlays.default # Makes ROS packages available via pkgs.rosPackages...
    # vendor                          # Adds our custom packages (see vendor section)
  ];

  # --- Packages ---
  # List the packages needed in the development shell
  packages =
    with pkgs; # Allows writing 'git' instead of 'pkgs.git'
    [
      # Essential dev tools
      git
      colcon # The ROS 2 build tool
      graphviz # Often needed for ROS visualization tools
      cairo # Dependency for some GUI libraries

      # --- Select ONE nixGL variant based on your GPU ---
      # Provides OpenGL support for GUI apps outside NixOS
      # Choose the one appropriate for your hardware/driver setup
      nixGL.auto.nixGLDefault # Often works
      # nixGL.nixGLIntel
      # nixGL.auto.nixGLNvidia
      # ... other variants
    ];
  # Add ROS 2 Humble packages
  # ++ (with pkgs.rosPackages.jazzy; [
  #   # Or change 'humble' to 'jazzy', 'noetic', etc.
  #   # --- ROS 2 Packages ---
  #   # Use buildEnv to group ROS packages and ensure their setup.sh is sourced
  #   (buildEnv {
  #     name = "ros-env"; # Name for this specific ROS package group
  #     paths = [
  #       # Core ROS libraries
  #       ros-core
  #       ament-cmake-core

  #       # Specific ROS packages for your project
  #       rosbridge-suite
  #       rplidar-ros # From our custom vendor overlay
  #       rviz2 # For visualization
  #       nav2-amcl # Navigation stack component
  #       slam-toolbox # SLAM algorithms
  #       tf2-ros # Transform library
  #       tf2-tools # TF debugging tools
  #       rqt-common-plugins # Useful RQT GUI tools
  #       rqt-tf-tree # RQT TF visualization
  #     ];
  #   })
  # ]);

  enterShell = ''
    echo -e "\e[38;5;208m______                                    _____ ";
    echo -e "| ___ \\                                  |____ |";
    echo -e "| |_/ /__ _ __ ___  ___ _   _ ___  __   __   / /";
    echo -e "|  __/ _ \\ '__/ __|/ _ \\ | | / __| \\ \\ / /   \\ \\";
    echo -e "| | |  __/ |  \\__ \\  __/ |_| \\__ \\  \\ V /.___/ /";
    echo -e "\\_|  \\___|_|  |___/\\___|\\__,_|___/   \\_/ \\____/ ";
    # echo -e "------------------------------------------------";
    echo -e "Remote Off-world Autonomous Robotics\e[0m";
  '';
}
