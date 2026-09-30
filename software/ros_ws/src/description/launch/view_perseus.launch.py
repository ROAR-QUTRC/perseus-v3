from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _joint_state_nodes(context):
    """The slider GUI (or its headless stand-in) and, for the bucket, its ram node.

    Without something publishing /joint_states, robot_state_publisher emits only the
    FIXED joints on /tf_static -- which does cover every sensor frame, since the mast
    and sensor mounts are all fixed -- but the four continuous wheel joints never
    appear and the tree is left incomplete. The headless stand-in (sliders:=false)
    publishes them at their defaults so the whole tree resolves, and takes any joint
    found on /payloads/joint_states from there, so the bucket follows its ros2_control
    stack (payloads bucket.launch.py) when that is running.

    With payload:=bucket the rams are not free joints: each is a function of the
    lift, tilt and jaw angles, and nothing else drives them here. So the slider node
    is rerouted through bucket_ram_follower: it publishes /joint_states_raw and reads
    /robot_description_sliders, and the follower republishes /joint_states with the
    ten ram joints exact, plus a description in which they are fixed so they get no
    slider. robot_state_publisher still reads the real description.
    """
    sliders = LaunchConfiguration("sliders")
    bucket = LaunchConfiguration("payload").perform(context) == "bucket"
    remap = (
        [
            ("joint_states", "joint_states_raw"),
            ("robot_description", "robot_description_sliders"),
        ]
        if bucket
        else []
    )
    actions = [
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            remappings=remap,
            output="screen",
            condition=IfCondition(sliders),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            parameters=[{"source_list": ["/payloads/joint_states"]}],
            remappings=remap,
            output="screen",
            condition=UnlessCondition(sliders),
        ),
    ]
    if bucket:
        actions.append(
            Node(
                package="description",
                executable="bucket_ram_follower.py",
                name="bucket_ram_follower",
                parameters=[{"viewer_mode": True}],
                output="screen",
            )
        )
        # The slider GUI shows radians; echo /joint_states_deg to read degrees.
        actions.append(
            Node(
                package="description",
                executable="joint_states_deg.py",
                parameters=[
                    {
                        "joints": [
                            "bucket_lift_joint",
                            "bucket_tilt_joint",
                            "bucket_jaw_joint",
                        ]
                    }
                ],
                output="screen",
            )
        )
    return actions


def generate_launch_description():
    """View the robot description, or just publish its TF tree.

    gui:=false drops RViz and leaves only the transforms, which is what you want
    over SSH -- the RViz here is wrapped in nixGL and needs a display, so it cannot
    come up on a headless machine at all. sliders:=true adds the joint state slider
    GUI; leave it off when a controller is driving the joints, or both publish
    /joint_states and the model jitters between them.
    """
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    gui = LaunchConfiguration("gui")
    hardware_plugin = LaunchConfiguration(
        "hardware_plugin", default="mock_components/GenericSystem"
    )
    can_bus = LaunchConfiguration("can_bus", default="")
    payload = LaunchConfiguration("payload", default="bucket")

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
            "payload": payload,
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description=(
                    "Launch RViz. Set false to publish the TF tree only, for "
                    "headless/SSH use"
                ),
            ),
            DeclareLaunchArgument(
                "sliders",
                default_value="false",
                description=(
                    "Launch the joint state slider GUI. Off by default so joints "
                    "follow /payloads/joint_states from the bucket's controller"
                ),
            ),
            DeclareLaunchArgument(
                "payload",
                default_value="bucket",
                description=(
                    "Payload attachment to include on the chassis. Set to "
                    "'none' to omit the bucket: frame mount, lift arms, bucket, "
                    "jaw and rams, with sliders for lift, tilt and jaw"
                ),
            ),
            rsp_launch,
            rviz,
            OpaqueFunction(function=_joint_state_nodes),
        ]
    )
