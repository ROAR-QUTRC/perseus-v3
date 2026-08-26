"""Rover half of the point cloud link: voxel downsample, then Draco compress.

For each of the two clouds worth sending to the base station -- the live Livox
scan and FAST-LIO's accumulated map -- this runs a `voxel_downsampler` to thin
the cloud, then a `point_cloud_transport` `republish` node to encode the thinned
cloud as Draco on `<downsampled topic>/draco`.

The base station runs point_cloud_decompress.launch.py to turn those Draco
topics back into PointCloud2.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Each stream is (node name prefix, raw input topic, downsampled base topic).
# The Draco topic is the base topic with a "/draco" suffix, which is what the
# draco publisher plugin appends of its own accord.
STREAMS = (
    ("livox", "/livox/lidar", "/livox/lidar/downsampled"),
    ("laser_map", "/Laser_map", "/Laser_map/downsampled"),
)


def draco_params(base_topic, encoder_config):
    """Build the {topic}.draco.* parameters the plugin declares, from a config section.

    draco_point_cloud_transport names its parameters after the topic it ends up
    advertising on (base_topic + "/draco"), with "/" replaced by ".". That topic is
    only known here, once the launch file has resolved base_topic, so these names
    can't be spelled out in the config file itself.

    force_quantization has to be set alongside quantization_POSITION, not left at the
    plugin's default: while it is false the plugin quantizes nothing, positions
    included, and quantization_POSITION has no observable effect -- the encoded
    payload comes out the same size as the raw input. See the config file for the
    measurements.
    """
    draco_topic = f"{base_topic}/draco"
    param_base = draco_topic.lstrip("/").replace("/", ".")
    return {
        f"{param_base}.quantization_POSITION": encoder_config["quantization_bits"],
        f"{param_base}.force_quantization": encoder_config["force_quantization"],
    }


def generate_launch_description():
    """Build the launch description for the rover-side compression pipeline."""
    config_file = os.path.join(
        get_package_share_directory("sensors"),
        "config",
        "point_cloud_compress.yaml",
    )
    with open(config_file) as f:
        config = yaml.safe_load(f)

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulated time"
    )
    use_sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}

    nodes = []
    for prefix, input_topic, base_topic in STREAMS:
        nodes.append(
            Node(
                package="sensors",
                executable="voxel_downsampler",
                name=f"{prefix}_voxel_downsampler",
                parameters=[
                    config_file,
                    {"input_topic": input_topic, "output_topic": base_topic},
                    use_sim_time,
                ],
                output="screen",
            )
        )
        encoder_config = config[f"{prefix}_draco_encoder"]["ros__parameters"]
        nodes.append(
            Node(
                package="point_cloud_transport",
                executable="republish",
                name=f"{prefix}_draco_encoder",
                parameters=[
                    {"in_transport": "raw", "out_transport": "draco"},
                    draco_params(base_topic, encoder_config),
                    use_sim_time,
                ],
                # `out` is only the base topic: the draco publisher plugin
                # advertises `out/draco`, so that is the name to remap.
                remappings=[
                    ("in", base_topic),
                    ("out/draco", f"{base_topic}/draco"),
                ],
                output="screen",
            )
        )

    return LaunchDescription([use_sim_time_arg, *nodes])
