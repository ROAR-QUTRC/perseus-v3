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
  # Use the pkgs from the
  packages =
    (builtins.attrValues workspace.devShellPkgs)
    ++ (builtins.attrValues workspace.standardPkgs)
    ++ ros2nixShell.nativeBuildInputs
    ++ ros2nixShell.buildInputs
    ++ [ ros_ws.standardPackages.workspace-shell-setup ];

  enterShell = ''
    # Pass the shell hook from the nix-ros-workspace shell to the devenv shell
    ${ros_ws.env.shellHook}

    # This will setup autocomplete for ros2 and colcon
    ${ros2nixShell.shellHook}

    echo -e "\e[38;5;208m______                                    _____ ";
    echo -e "| ___ \\                                  |____ |";
    echo -e "| |_/ /__ _ __ ___  ___ _   _ ___  __   __   / /";
    echo -e "|  __/ _ \\ '__/ __|/ _ \\ | | / __| \\ \\ / /   \\ \\";
    echo -e "| | |  __/ |  \\__ \\  __/ |_| \\__ \\  \\ V /.___/ /";
    echo -e "\\_|  \\___|_|  |___/\\___|\\__,_|___/   \\_/ \\____/ ";
    echo -e "QUTRC - Remote Off-world Autonomous Robotics\e[0m";
  '';
}
