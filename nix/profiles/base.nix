{ pkgs, config, ... }:
{
  env.ROS_DOMAIN_ID = 42;

  enterShell = ''
    # enable coloured ros2 launch output
    export RCUTILS_COLORIZED_OUTPUT=1

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
