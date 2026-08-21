{ pkgs, ... }:
let
  prod_ws = (import ../ros-workspace.nix { inherit pkgs; }).mkWorkspace {
    inherit (pkgs) ros;
    name = "prod";
  };
in
{
  packages = [
    prod_ws
  ];

  enterShell = ''
    # Pass the shell hook from the nix-ros-workspace shell to the devenv shell
    ${prod_ws.env.shellHook}

    echo -e "\033[35m______                                    _____ ";
    echo -e "| ___ \\                                  |____ |";
    echo -e "| |_/ /__ _ __ ___  ___ _   _ ___  __   __   / /";
    echo -e "|  __/ _ \\ '__/ __|/ _ \\ | | / __| \\ \\ / /   \\ \\";
    echo -e "| | |  __/ |  \\__ \\  __/ |_| \\__ \\  \\ V /.___/ /";
    echo -e "\\_|  \\___|_|  |___/\\___|\\__,_|___/   \\_/ \\____/ ";
    echo -e "Production shell >>>\e[0m";
  '';
}
