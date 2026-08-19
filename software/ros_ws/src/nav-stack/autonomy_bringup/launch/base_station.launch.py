"""Base station view: RViz only, for an operator machine watching a robot elsewhere.

Nothing here drives the robot. This launches the visualisation side alone, so it can be
run on a laptop that shares a ROS domain with the rover while the sensing, estimation and
health monitoring all run on the rover itself.

The default config docks the Topic Health panel from health_check, which tabulates
the rate, bandwidth and staleness of each monitored topic from the rover's health_monitor.
That table is the quickest way to tell a link problem from a node that has died.

Arguments:
    rviz_config   path to an RViz config, to open a different view without editing this file
    use_sim_time  set true when following a simulated robot, so displays honour /clock
    use_nixgl     wrap RViz in nixGL for GPU access; true matches the other launch files
                  in this repo, false runs rviz2 directly on a machine with working drivers
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_nixgl = LaunchConfiguration("use_nixgl")

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("autonomy_bringup"), "rviz", "base_station.rviz"]
        ),
        description="RViz config to open",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use /clock instead of wall time",
    )
    use_nixgl_arg = DeclareLaunchArgument(
        "use_nixgl",
        default_value="true",
        description="Wrap RViz in nixGL for GPU access",
    )

    # The environment below matches description/launch/view_perseus.launch.py: RViz needs
    # xcb rather than wayland to render reliably here, and the deeper QoS history keeps
    # displays from dropping messages on a busy link.
    rviz_env = {
        "NIXPKGS_ALLOW_UNFREE": "1",
        "QT_QPA_PLATFORM": "xcb",
        "QT_SCREEN_SCALE_FACTORS": "1",
        "RMW_QOS_POLICY_HISTORY": "keep_last",
        "RMW_QOS_POLICY_DEPTH": "100",
    }

    rviz_nixgl = ExecuteProcess(
        condition=IfCondition(use_nixgl),
        cmd=[
            "nix",
            "run",
            "--impure",
            "github:nix-community/nixGL",
            "--",
            "rviz2",
            "-d",
            rviz_config,
        ],
        output="screen",
        additional_env=rviz_env,
    )

    # Plain node rather than ExecuteProcess so use_sim_time can be passed as a parameter;
    # the nixGL path above cannot take one, since it is a bare command line.
    rviz_plain = Node(
        condition=UnlessCondition(use_nixgl),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        additional_env=rviz_env,
    )

    return LaunchDescription(
        [
            rviz_config_arg,
            use_sim_time_arg,
            use_nixgl_arg,
            rviz_nixgl,
            rviz_plain,
        ]
    )
