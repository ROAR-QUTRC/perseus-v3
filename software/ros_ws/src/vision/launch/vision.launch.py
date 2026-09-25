"""Launch the vision pipeline, either composed with the camera driver or node by node.

`enable_sensors:=` picks between two shapes, and the difference is not cosmetic:

  enable_sensors:=false (default)  The camera is assumed to be running already -- from
                                   sensors/sensors.launch.py in its own terminal, or
                                   Gazebo in the simulator. Every enabled node runs as its
                                   own process, so every image crosses the middleware.
  enable_sensors:=true             This file brings up sensors/realsense.launch.py itself
                                   and loads every enabled node as a *component* into the
                                   container that file creates, with intra-process comms,
                                   so the images are handed over by pointer.

The composed path exists because the infra pair is 848x480 Y8 at 30 fps -- 407 KB a frame,
24 MB/s for the two together. Run as a separate process every byte of that is serialised
through the middleware, and measured on this rover 86% of frames never arrived: 29.8 fps at
the camera's own frame counter against 4.2 Hz on the topic, with infra1 and infra2 drifting
to different rates so libviso2 was matching frames that did not correspond to each other.
Loaded into the driver's container both streams measure a clean 30 Hz.

The node-by-node path is the baseline to compare against, and the only one that works
without a RealSense attached. It is the default because nothing in this repo orchestrates
bringup -- the drivers are started by hand -- and defaulting to true would open the camera a
second time alongside an existing one and fail with "Device or resource busy".

Both shapes are built from the one NODES table below, with the same names and the same
vision.yaml, so they run the same nodes on the same parameters and differ only in placement.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

# Must match realsense.launch.py's DEFAULT_CONTAINER_NAME. That file creates the container,
# this one loads into it by name, and a mismatch does not raise: LoadComposableNodes simply
# waits for a container that never appears.
DEFAULT_CONTAINER_NAME = "sensor_container"

# (launch argument, node name, executable, component plugin). The names are not free
# choices: vision.yaml keys each node's parameters under a top-level block of the same name
# at the root namespace, so renaming one would silently drop every parameter and leave it on
# its built-in defaults -- for stereo_odometry that includes publish_tf, which must stay
# false while the EKF owns odom -> base_link.
NODES = [
    ("stereo_odometry", "stereo_odometry", "stereo_odometry", "vision::StereoOdometry"),
    ("aruco", "aruco_detector", "aruco_detector_node", "vision::ArucoDetector"),
    ("cube", "cube_detector", "cube_detector", "vision::CubeDetector"),
    (
        "overlay",
        "detection_overlay",
        "detection_overlay_node",
        "vision::DetectionOverlay",
    ),
]


def launch_setup(context, *args, **kwargs):
    """Build the enabled nodes in whichever shape `enable_sensors` selects.

    An OpaqueFunction because the toggles decide which actions exist at all, so they are
    needed as Python values rather than substitutions.
    """
    config_file = os.path.join(
        get_package_share_directory("vision"), "config", "vision.yaml"
    )
    parameters = [config_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}]

    enabled = [
        (name, executable, plugin)
        for toggle, name, executable, plugin in NODES
        if LaunchConfiguration(toggle).perform(context).lower() == "true"
    ]
    enable_sensors = (
        LaunchConfiguration("enable_sensors").perform(context).lower() == "true"
    )

    if not enable_sensors:
        return [
            Node(
                package="vision",
                executable=executable,
                name=name,
                namespace="",
                parameters=parameters,
                output="screen",
            )
            for name, executable, _ in enabled
        ]

    container_name = LaunchConfiguration("container_name")
    actions = [
        # Creates the component container the load below targets. No use_sim_time to pass:
        # the driver timestamps from the camera's own clock regardless, and
        # realsense.launch.py declares no such argument to receive it.
        GroupAction(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [
                                FindPackageShare("sensors"),
                                "launch",
                                "realsense.launch.py",
                            ]
                        )
                    ),
                    # Pinned rather than left to realsense.launch.py's identical default,
                    # and this is not belt-and-braces. GroupAction(scoped=True) forwards the
                    # enclosing configurations inward, and `ros2 launch` accepts any k:=v
                    # the file never declared, so a stray argument would reach this include
                    # and could rename the container out from under the load below --
                    # LoadComposableNodes waits without a timeout and logs nothing above
                    # DEBUG, so the symptom is a launch that hangs in silence. An explicit
                    # argument wins inside the scope.
                    launch_arguments={"container_name": container_name}.items(),
                )
            ],
            scoped=True,
        )
    ]
    if enabled:
        actions.append(
            LoadComposableNodes(
                target_container=container_name,
                composable_node_descriptions=[
                    ComposableNode(
                        package="vision",
                        plugin=plugin,
                        name=name,
                        namespace="",
                        parameters=parameters,
                        extra_arguments=[{"use_intra_process_comms": True}],
                    )
                    for name, _, plugin in enabled
                ],
            )
        )
    return actions


def generate_launch_description():
    """Build the launch description for the vision pipeline."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulated time"
            ),
            DeclareLaunchArgument(
                "enable_sensors",
                default_value="false",
                # Constrained because launch_setup reads this with a strict `== "true"`,
                # which -- unlike IfCondition -- does not accept 1/on/yes, so without
                # choices an `enable_sensors:=1` would read as false and silently skip
                # the camera.
                choices=["true", "false"],
                description="Bring the RealSense driver up from this file and load the "
                "vision nodes into its component container (30 Hz on the infra pair). "
                "False runs each node as its own process against an already-running "
                "camera (roughly 4 Hz on the infra pair). See the module docstring.",
            ),
            DeclareLaunchArgument(
                "container_name",
                default_value=DEFAULT_CONTAINER_NAME,
                description="Component container the camera driver runs in and the vision "
                "nodes load into. Used only when enable_sensors:=true.",
            ),
            DeclareLaunchArgument(
                "stereo_odometry",
                default_value="true",
                description="Enable stereo visual odometry on the infra pair",
            ),
            DeclareLaunchArgument(
                "aruco", default_value="true", description="Enable the ArUco detector"
            ),
            DeclareLaunchArgument(
                "cube", default_value="false", description="Enable the cube detector"
            ),
            DeclareLaunchArgument(
                "overlay",
                default_value="true",
                description="Enable the detection overlay",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
