from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """View the robot description, or just publish its TF tree.

    gui:=false drops RViz and the joint state slider GUI and leaves only the
    transforms, which is what you want over SSH -- the RViz here is wrapped in
    nixGL and needs a display, so it cannot come up on a headless machine at all.
    """
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    gui = LaunchConfiguration("gui")
    hardware_plugin = LaunchConfiguration(
        "hardware_plugin", default="mock_components/GenericSystem"
    )
    can_bus = LaunchConfiguration("can_bus", default="")

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("perseus"),
                        "launch",
                        "robot_state_publisher.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "hardware_plugin": hardware_plugin,
            "can_bus": can_bus,
        }.items(),
    )

    # RViz with nixGL support
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("description"), "rviz", "view_perseus.rviz"]
    )
    rviz = ExecuteProcess(
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
        additional_env={
            "NIXPKGS_ALLOW_UNFREE": "1",
            "QT_QPA_PLATFORM": "xcb",
            "QT_SCREEN_SCALE_FACTORS": "1",
            "ROS_NAMESPACE": "/",
            "RMW_QOS_POLICY_HISTORY": "keep_last",
            "RMW_QOS_POLICY_DEPTH": "100",
        },
        condition=IfCondition(gui),
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui = ExecuteProcess(
        cmd=["ros2", "run", "joint_state_publisher_gui", "joint_state_publisher_gui"],
        output="screen",
        condition=IfCondition(gui),
    )

    # Headless stand-in for the slider GUI. Without something publishing
    # /joint_states, robot_state_publisher emits only the FIXED joints on
    # /tf_static -- which does cover every sensor frame, since the mast and
    # sensor mounts are all fixed -- but the four continuous wheel joints never
    # appear and the tree is left incomplete. This publishes them at their
    # defaults so the whole tree resolves, at no cost on a headless box.
    joint_state_publisher = ExecuteProcess(
        cmd=["ros2", "run", "joint_state_publisher", "joint_state_publisher"],
        output="screen",
        condition=UnlessCondition(gui),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description=(
                    "Launch RViz and the joint state slider GUI. Set false to "
                    "publish the TF tree only, for headless/SSH use"
                ),
            ),
            rsp_launch,
            rviz,
            joint_state_publisher_gui,
            joint_state_publisher,
        ]
    )
