from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os
import yaml


def start_after_success(stage_name, actions):
    """Start dependent actions only when a prerequisite exits successfully."""

    def handle_exit(event, _context):
        if event.returncode != 0:
            reason = f"{stage_name} failed with exit code {event.returncode}"
            return [
                LogInfo(msg=f"ERROR: {reason}"),
                EmitEvent(event=Shutdown(reason=reason)),
            ]

        return [
            LogInfo(msg=f"{stage_name} is ready; starting the next stage"),
            *actions,
        ]

    return handle_exit


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rmw_implementation",
            default_value="rmw_fastrtps_cpp",
            description="ROS middleware used by all simulation processes",
        )
    )

    # Initialize Arguments
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    rmw_implementation = LaunchConfiguration("rmw_implementation")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("payloads"), "config", "arm.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Semantic Robot Description (SRDF)
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("payloads"), "config", "arm.srdf"]),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content, value_type=str
        )
    }

    # Load Kinematics
    kinematics_yaml = load_yaml("payloads", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Load Servo Config
    servo_yaml = load_yaml("payloads", "config/servo.yaml")

    # Load Joint Limits
    joint_limits_yaml = load_yaml("payloads", "config/joint_limits.yaml")
    robot_description_planning = {"robot_description_planning": joint_limits_yaml}

    # MoveIt Configuration
    ompl_planning_yaml = load_yaml("payloads", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
            ],
            "start_state_max_bounds_error": 0.1,
        },
    }
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    moveit_controllers = {
        "moveit_simple_controller_manager": load_yaml(
            "payloads", "config/moveit_controllers.yaml"
        ),
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "use_sim_time": use_sim_time,
        "joint_state_topic": "/joint_states",
    }

    # Nodes

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {
                "use_robot_description_topic": False,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    # 2. ROS2 Control
    ros2_controllers_path = os.path.join(
        get_package_share_directory("payloads"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ros2_controllers_path,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("robot_description", "/robot_description")],
        output="both",
    )

    # 3. Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "20",
            "--service-call-timeout",
            "10",
            "--switch-timeout",
            "10",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    servo_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "servo_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "20",
            "--service-call-timeout",
            "10",
            "--switch-timeout",
            "10",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    wrist_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "wrist_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "20",
            "--service-call-timeout",
            "10",
            "--switch-timeout",
            "10",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "20",
            "--service-call-timeout",
            "10",
            "--switch-timeout",
            "10",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 4. Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    # 5. MoveIt Servo Node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            {"moveit_servo": servo_yaml},
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    wrist_velocity_bridge_node = Node(
        package="payloads",
        executable="wrist_velocity_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    gripper_velocity_bridge_node = Node(
        package="payloads",
        executable="gripper_velocity_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 6. RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("payloads"), "config", "moveit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_rviz),
    )

    # 7. Static Transform Publisher (world -> plate)
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "plate"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    start_servo_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=start_after_success(
                "Joint-state broadcaster",
                [servo_controller_spawner],
            ),
        )
    )

    start_wrist_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=servo_controller_spawner,
            on_exit=start_after_success(
                "Servo trajectory controller",
                [wrist_controller_spawner],
            ),
        )
    )

    start_gripper_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=wrist_controller_spawner,
            on_exit=start_after_success(
                "Wrist trajectory controller",
                [gripper_controller_spawner],
            ),
        )
    )

    start_moveit = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=start_after_success(
                "Gripper trajectory controller",
                [
                    move_group_node,
                    servo_node,
                    wrist_velocity_bridge_node,
                    gripper_velocity_bridge_node,
                    rviz_node,
                ],
            ),
        )
    )

    nodes_to_start = [
        SetEnvironmentVariable(
            name="RMW_IMPLEMENTATION",
            value=rmw_implementation,
        ),
        robot_state_publisher_node,
        ros2_control_node,
        static_tf_node,
        joint_state_broadcaster_spawner,
        start_servo_controller,
        start_wrist_controller,
        start_gripper_controller,
        start_moveit,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
