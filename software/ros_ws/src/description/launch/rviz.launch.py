"""RViz on its own, through nixGL, as view_perseus.launch.py runs it.

For looking at a stack that is already running elsewhere (on the rover, or another
launch on this machine). config:=<path> opens a saved .rviz file; left empty, RViz
opens with its defaults.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _rviz(context):
    """Built at launch time, so an empty config leaves out -d rather than passing ''."""
    config = context.perform_substitution(LaunchConfiguration("config"))
    rviz_args = ["-d", config] if config else []
    return [
        ExecuteProcess(
            cmd=[
                "nix",
                "run",
                "--impure",
                "github:nix-community/nixGL",
                "--",
                "rviz2",
                *rviz_args,
            ],
            output="screen",
            additional_env={
                "NIXPKGS_ALLOW_UNFREE": "1",
                "QT_QPA_PLATFORM": "xcb",
                "QT_SCREEN_SCALE_FACTORS": "1",
                "ROS_NAMESPACE": "/",
                "RMW_QOS_POLICY_HISTORY": "keep_last",
                "RMW_QOS_POLICY_DEPTH": "100",
            },
        )
    ]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "config",
            default_value="",
            description="RViz config file to open; empty opens RViz with its defaults",
        ),
    ]

    return LaunchDescription(arguments + [OpaqueFunction(function=_rviz)])
