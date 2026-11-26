"""Base station half of the point cloud link: Draco decompress back to raw.

Mirror of point_cloud_compress.launch.py. For each Draco topic the rover sends,
a `point_cloud_transport` `republish` node decodes it back into a PointCloud2 on
a `/decompressed` topic, which RViz can display directly.

No custom node is involved -- `republish` handles both directions, it is just
pointed the other way.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Each stream is (node name prefix, downsampled base topic). The rover publishes
# the compressed cloud on "<base topic>/draco"; the decoded cloud comes out on
# "<base topic>/decompressed", kept distinct from the rover's own raw topic name
# so the two never collide if both are visible on the network.
STREAMS = (
    ("livox", "/livox/lidar/downsampled"),
    ("laser_map", "/Laser_map/downsampled"),
)


def generate_launch_description():
    """Build the launch description for the base-station decompression pipeline."""
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulated time"
    )
    use_sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}

    nodes = [
        Node(
            package="point_cloud_transport",
            executable="republish",
            name=f"{prefix}_draco_decoder",
            parameters=[
                {"in_transport": "draco", "out_transport": "raw"},
                use_sim_time,
            ],
            # `in` is only the base topic: the draco subscriber plugin listens on
            # `in/draco`, so that is the name to remap. The raw publisher adds no
            # suffix, so `out` is remapped directly.
            remappings=[
                ("in/draco", f"{base_topic}/draco"),
                ("out", f"{base_topic}/decompressed"),
            ],
            output="screen",
        )
        for prefix, base_topic in STREAMS
    ]

    return LaunchDescription([use_sim_time_arg, *nodes])
