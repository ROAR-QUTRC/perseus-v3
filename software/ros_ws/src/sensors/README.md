# Perseus Sensors - IMU and Topic Nodes

This package provides ROS2 nodes for **IMU bias estimation and correction**, **dynamic topic remapping** for downsampling sensor data, and **point cloud thinning** for the rover-to-base-station link.

Included nodes:

1. **BiasEstimator** – estimates IMU gyro bias when the robot is stationary.
2. **BiasRemover** – removes the estimated bias from IMU readings.
3. **TopicRemapper** – dynamically detects a topic type and republishes it at a lower frequency.
4. **VoxelDownsampler** – thins a point cloud to at most one point per voxel cell.

---

> ⚠️ **Open-Source Attribution:**  
> The **BiasEstimator** and **BiasRemover** ROS2 nodes in this package are adapted from the
> [ros-perception/imu_pipeline](https://github.com/ros-perception/imu_pipeline/tree/ros2) project.  
> In particular, the bias removal logic is based on the implementation in `imu_processors/src/imu_bias_remover.cpp` from that repository, with extensions for stationary detection, configurable parameters, and composable node support in the Perseus platform.

## 1. BiasEstimator Node

**Node Name:** `imu_bias_estimator`
**Package:** `sensors`
**Plugin (Composable):** `imu_processors::BiasEstimator`

### Description

Estimates the bias of the IMU angular velocity (`gyro`) when the robot is stationary. Stationary detection can use **odom**, **cmd_vel**, or both.

### Subscribed Topics

| Topic           | Type                                    | Description                                        |
| --------------- | --------------------------------------- | -------------------------------------------------- |
| `imu_in_topic`  | `sensor_msgs/Imu`                       | Raw IMU data                                       |
| `cmd_vel_topic` | `geometry_msgs/Twist` or `TwistStamped` | (Optional) Robot velocity for stationary detection |
| `odom_topic`    | `nav_msgs/Odometry`                     | (Optional) Robot odometry for stationary detection |

### Published Topics

| Topic            | Type                           | Description                     |
| ---------------- | ------------------------------ | ------------------------------- |
| `bias_out_topic` | `geometry_msgs/Vector3Stamped` | Estimated angular velocity bias |

### Parameters

| Parameter           | Type   | Default     | Description                                 |
| ------------------- | ------ | ----------- | ------------------------------------------- |
| `imu_in_topic`      | string | `"imu"`     | Input IMU topic                             |
| `bias_out_topic`    | string | `"bias"`    | Output bias topic                           |
| `cmd_vel_topic`     | string | `"cmd_vel"` | Cmd_vel topic for stationary detection      |
| `odom_topic`        | string | `"odom"`    | Odom topic for stationary detection         |
| `use_cmd_vel`       | bool   | `false`     | Enable cmd_vel for stationary detection     |
| `use_odom`          | bool   | `false`     | Enable odometry for stationary detection    |
| `use_stamped`       | bool   | `false`     | Use `TwistStamped` instead of `Twist`       |
| `stationary_mode`   | string | `"OR"`      | Stationary policy: `"OR"` or `"AND"`        |
| `accumulator_alpha` | double | `0.01`      | EWMA alpha for bias accumulation            |
| `cmd_vel_threshold` | double | `0.001`     | Cmd_vel threshold for stationary detection  |
| `odom_threshold`    | double | `0.001`     | Odometry threshold for stationary detection |
| `estimator_rate_hz` | double | `50.0`      | Bias estimation and publishing frequency    |

---

## 2. BiasRemover Node

**Node Name:** `imu_bias_remover`
**Package:** `sensors`
**Plugin (Composable):** `imu_processors::BiasRemover`

### Description

Removes the latest bias from the IMU angular velocity and republishes corrected IMU messages. Can throttle output rate.

### Subscribed Topics

| Topic           | Type                           | Description                         |
| --------------- | ------------------------------ | ----------------------------------- |
| `imu_in_topic`  | `sensor_msgs/Imu`              | Raw IMU data                        |
| `bias_in_topic` | `geometry_msgs/Vector3Stamped` | Bias estimated from `BiasEstimator` |

### Published Topics

| Topic           | Type              | Description        |
| --------------- | ----------------- | ------------------ |
| `imu_out_topic` | `sensor_msgs/Imu` | Bias-corrected IMU |

### Parameters

| Parameter           | Type   | Default                | Description                                       |
| ------------------- | ------ | ---------------------- | ------------------------------------------------- |
| `imu_in_topic`      | string | `"imu"`                | Input IMU topic                                   |
| `imu_out_topic`     | string | `"imu_bias_corrected"` | Corrected IMU output topic                        |
| `bias_in_topic`     | string | `"bias"`               | Input bias topic                                  |
| `zero_when_no_bias` | bool   | `false`                | If no bias received, set angular velocity to zero |
| `output_rate_hz`    | double | `50.0`                 | Throttle publishing rate                          |

---

## 3. TopicRemapper Node

**Node Name:** `topic_remapper`
**Package:** `sensors`

### Description

Dynamically detects the message type of an input topic and republishes it at a lower frequency. Useful for downsampling high-frequency topics such as IMU or LiDAR.

### Subscribed Topics

| Topic         | Type    | Description           |
| ------------- | ------- | --------------------- |
| `input_topic` | dynamic | Source topic to remap |

### Published Topics

| Topic          | Type    | Description                       |
| -------------- | ------- | --------------------------------- |
| `output_topic` | dynamic | Remapped topic at lower frequency |

### Parameters

| Parameter             | Type   | Default      | Description               |
| --------------------- | ------ | ------------ | ------------------------- |
| `input_topic`         | string | -            | Topic to remap (required) |
| `output_topic`        | string | `"remapped"` | Remapped output topic     |
| `reduction_frequency` | double | `10.0`       | Output frequency in Hz    |

---

## 4. VoxelDownsampler Node

**Node Name:** `voxel_downsampler`
**Package:** `sensors`

### Description

Republishes a point cloud keeping at most one point per voxel cell. Surviving points are copied
through byte for byte, so whatever fields the input carries (intensity, timestamp, tag, line, …)
come out unchanged — only the point count drops. Non-finite points are dropped.

Compression is **not** done here: it is left to `point_cloud_transport`'s `republish` node, which
sits downstream (see below).

### Subscribed Topics

| Topic         | Type                      | Description  |
| ------------- | ------------------------- | ------------ |
| `input_topic` | `sensor_msgs/PointCloud2` | Source cloud |

### Published Topics

| Topic          | Type                      | Description   |
| -------------- | ------------------------- | ------------- |
| `output_topic` | `sensor_msgs/PointCloud2` | Thinned cloud |

### Parameters

| Parameter      | Type   | Default                      | Description                       |
| -------------- | ------ | ---------------------------- | --------------------------------- |
| `input_topic`  | string | `"/livox/lidar"`             | Source cloud topic                |
| `output_topic` | string | `"/livox/lidar/downsampled"` | Thinned cloud topic               |
| `voxel_size_m` | double | `0.1`                        | Voxel cube edge length, in metres |

---

## Point cloud link (rover ↔ base station)

Two launch files carry the Livox scan and FAST-LIO's accumulated map across the link, one for each
direction of the codec. Both ends need `draco_point_cloud_transport` installed — `republish` loads
the plugin by name at runtime, so a missing plugin only shows up as a launch-time error.

**On the rover** — thin, then encode:

```bash
ros2 launch sensors point_cloud_compress.launch.py
```

| Raw cloud      | Thinned                    | Sent over the link               |
| -------------- | -------------------------- | -------------------------------- |
| `/livox/lidar` | `/livox/lidar/downsampled` | `/livox/lidar/downsampled/draco` |
| `/Laser_map`   | `/Laser_map/downsampled`   | `/Laser_map/downsampled/draco`   |

**On the base station** — decode back to `PointCloud2` for RViz:

```bash
ros2 launch sensors point_cloud_decompress.launch.py
```

| Received                         | Decoded                                 |
| -------------------------------- | --------------------------------------- |
| `/livox/lidar/downsampled/draco` | `/livox/lidar/downsampled/decompressed` |
| `/Laser_map/downsampled/draco`   | `/Laser_map/downsampled/decompressed`   |

Voxel sizes and Draco quantization both live in `config/point_cloud_compress.yaml`, one section
per stream. Topic names live in the launch file itself, since it also needs them to wire up the
matching `republish` nodes.

| Parameter            | Default       | Description                                                               |
| -------------------- | ------------- | ------------------------------------------------------------------------- |
| `voxel_size_m`       | `0.2` / `0.3` | Voxel cube edge length, in metres (per stream)                            |
| `quantization_bits`  | `12`          | Bits per x/y/z coordinate for Draco position quantization (0-31)          |
| `force_quantization` | `true`        | Required for `quantization_bits` to take effect; false disables all of it |

Leave `force_quantization` on unless a consumer needs the non-position fields bit-exact. The
plugin defaults it to false, and while it is false it quantizes nothing at all -- positions
included -- so Draco returns a payload the same size as its input. Measured on the accumulated
map at 12 bits: 390336 B in, 390421 B out with it false, 89658 B out with it true. The live scan
improved from 1.99x to 6.73x. Decoded positions land within 5.8 mm of the input, against a
300 mm voxel grid.

Note that Draco reorders points (it deduplicates while encoding), so the decoded cloud is not
index-aligned with the input. Compare the two as point sets, not element-wise.

> ℹ️ `republish` resolves its `in`/`out` topics _before_ the transport plugin appends its suffix, so
> the remap has to name the suffixed topic: `out/draco:=…` when encoding, `in/draco:=…` when
> decoding. Remapping bare `out`/`in` for the Draco side silently does nothing.

`quantization_bits` is the one Draco knob exposed through the config file; it's set on the encoder
node under a name derived from the topic it publishes on (`/` replaced by `.`), e.g.
`livox.lidar.downsampled.draco.quantization_POSITION` for the Livox stream. Draco's other knobs
(encode speed, deduplication, ...) are left at the plugin's defaults; `ROS2 param list
/livox_draco_encoder` shows the full set if you need to tune those too.
