{ pkgs, ... }:
let
  workspace = (import ../ros-workspace.nix { inherit pkgs; });
  ros_ws = workspace.mkWorkspace {
    inherit (pkgs) ros;
    name = "dev";
    isDev = true;
  };
  ros2nixShell = import ../extra-packages/ros-packages/perseus/shell.nix {
    inherit pkgs;
  };
in
{
  packages =
    # dev and standard pkgs propagated through the nix ros workspace
    (builtins.attrValues workspace.devShellPkgs)
    ++ (builtins.attrValues workspace.standardPkgs)
    # run time dependencies from the ros2nix shell
    ++ ros2nixShell.buildInputs
    ++ ros2nixShell.propagatedBuildInputs
    # build time dependencies from the ros2nix shell
    ++ ros2nixShell.nativeBuildInputs
    ++ ros2nixShell.propagatedNativeBuildInputs
    # cli autocomplete run by the shell hook
    ++ [ ros_ws.standardPackages.workspace-shell-setup ];

  enterShell = ''
    # Pass the shell hook from the nix-ros-workspace shell to the devenv shell
    ${ros_ws.env.shellHook}

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
    #
    # This belongs to the dev profile only: prod builds every package with nix, so there
    # is no colcon install/ tree for it to prefer.
    export PATH="$ROS_WORKSPACE_ENV_PATH/bin:$PATH"

    echo -e "\e[38;5;208m______                                    _____ ";
    echo -e "| ___ \\                                  |____ |";
    echo -e "| |_/ /__ _ __ ___  ___ _   _ ___  __   __   / /";
    echo -e "|  __/ _ \\ '__/ __|/ _ \\ | | / __| \\ \\ / /   \\ \\";
    echo -e "| | |  __/ |  \\__ \\  __/ |_| \\__ \\  \\ V /.___/ /";
    echo -e "\\_|  \\___|_|  |___/\\___|\\__,_|___/   \\_/ \\____/ ";
    echo -e "QUTRC - Remote Off-world Autonomous Robotics\e[0m";
  '';
}
