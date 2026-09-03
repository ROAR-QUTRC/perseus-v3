/// @file voxel_downsampler.cpp
/// @brief Implementation of the voxel-grid downsampling node.

#include "sensors/voxel_downsampler/voxel_downsampler.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_set>

namespace sensors {
namespace {
/// @brief Queue depth used for the input and output topics.
constexpr int QOS_DEPTH = 10;

/// @brief Bits of voxel-grid range packed per axis into a hash key.
///
/// 2^20 cells either side of the origin, which even at a 1 cm voxel size covers
/// a ~10 km cube -- far beyond any range this robot operates at.
constexpr int32_t VOXEL_KEY_BITS_PER_AXIS = 21;
constexpr int64_t VOXEL_KEY_AXIS_MASK =
    (int64_t(1) << VOXEL_KEY_BITS_PER_AXIS) - 1;
constexpr int32_t VOXEL_KEY_AXIS_OFFSET = 1 << (VOXEL_KEY_BITS_PER_AXIS - 1);

/// @brief Looks up one field in a cloud, if it is present with the expected
/// datatype.
/// @param msg Cloud to search.
/// @param name Field name to look for.
/// @param datatype Required sensor_msgs/PointField datatype constant.
/// @return The field's byte offset, or nullopt if absent or a different type.
std::optional<std::size_t> find_field(const sensor_msgs::msg::PointCloud2 &msg,
                                      const std::string &name,
                                      uint8_t datatype) {
  for (const auto &field : msg.fields) {
    if (field.name == name && field.datatype == datatype && field.count == 1) {
      return static_cast<std::size_t>(field.offset);
    }
  }
  return std::nullopt;
}
} // namespace

VoxelDownsampler::VoxelDownsampler(const rclcpp::NodeOptions &options)
    : rclcpp::Node("voxel_downsampler", options) {
  std::string input_topic;
  std::string output_topic;
  _load_parameters(input_topic, output_topic);

  _point_cloud_subscription =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
          input_topic, QOS_DEPTH,
          std::bind(&VoxelDownsampler::_point_cloud_callback, this,
                    std::placeholders::_1));

  _publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, QOS_DEPTH);

  _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

  RCLCPP_INFO(this->get_logger(),
              "Voxel downsampler: %s -> %s (voxel size %.3f m, height filter "
              "%s at %.3f m in %s)",
              input_topic.c_str(), output_topic.c_str(), _voxel_size_m,
              _height_filter_enabled ? "enabled" : "disabled", _max_height_m,
              _height_filter_frame.c_str());
}

void VoxelDownsampler::_load_parameters(std::string &input_topic_out,
                                        std::string &output_topic_out) {
  input_topic_out = this->declare_parameter("input_topic", DEFAULT_INPUT_TOPIC);
  output_topic_out =
      this->declare_parameter("output_topic", DEFAULT_OUTPUT_TOPIC);
  // Position and intensity survive merging points; per-return metadata does
  // not. Set this to an empty list to carry every field the input has, at the
  // cost of the Draco encoder (see the comment where the output layout is
  // built).
  _keep_fields = this->declare_parameter<std::vector<std::string>>(
      "keep_fields", std::vector<std::string>{"x", "y", "z", "intensity"});
  _voxel_size_m = this->declare_parameter("voxel_size_m", DEFAULT_VOXEL_SIZE_M);

  // A non-positive voxel size would make the grid coordinate computation divide
  // by zero (or invert the grid), so clamp it back to the default rather than
  // publishing garbage.
  if (!(_voxel_size_m > 0.0)) {
    RCLCPP_WARN(this->get_logger(),
                "voxel_size_m must be positive, got %.3f -- falling back to "
                "%.3f m",
                _voxel_size_m, DEFAULT_VOXEL_SIZE_M);
    _voxel_size_m = DEFAULT_VOXEL_SIZE_M;
  }

  // Everything above max_height_m, measured in height_filter_frame, is dropped
  // before it reaches the voxel grid -- the highest frame on the robot
  // (livox_frame, the tilted Livox mount) plus a clearance margin above it, so
  // that mast/antenna returns and anything further overhead are neither
  // detected nor transmitted. Measured in odom rather than in the cloud's own
  // (possibly tilted) sensor frame so the cutoff is a fixed height off the
  // ground the robot started on.
  _height_filter_enabled =
      this->declare_parameter("height_filter_enabled", true);
  _height_filter_frame = this->declare_parameter("height_filter_frame",
                                                 DEFAULT_HEIGHT_FILTER_FRAME);
  _max_height_m = this->declare_parameter("max_height_m", DEFAULT_MAX_HEIGHT_M);
}

void VoxelDownsampler::_point_cloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  const std::optional<position_offsets_t> resolved =
      _resolve_position_offsets(*msg);
  if (!resolved.has_value()) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Input cloud has no float32 x/y/z fields, ignoring.");
    return;
  }
  const position_offsets_t offsets = resolved.value();

  const std::optional<tf2::Transform> height_filter_transform =
      _resolve_height_filter_transform(msg->header.frame_id);

  // Trust data.size() over width*height: a cloud that declares more points than
  // it carries would otherwise walk off the end of the buffer.
  const std::size_t point_step = msg->point_step;
  const std::size_t point_count =
      std::min(static_cast<std::size_t>(msg->width) * msg->height,
               msg->data.size() / point_step);

  // The output carries only the fields that still mean something once points
  // have been merged, which by default is position and intensity. A Livox cloud
  // also arrives with tag, line and a float64 timestamp; those describe one
  // specific return, and the point published here stands for several, so
  // keeping them would be stating something untrue about the survivor.
  //
  // It is also what lets the cloud be compressed. Draco encodes a point cloud
  // as integer attributes or quantized floats, and the quantization the
  // transport exposes covers float32 attribute classes only -- a float64 field
  // is neither, so the encoder rejects the whole cloud with "Invalid encoding
  // method" and the Draco topic never publishes. Dropping the field also takes
  // point_step from 26 bytes to 16 before Draco even runs.
  const std::vector<sensor_msgs::msg::PointField> out_fields =
      _select_output_fields(*msg);
  std::size_t out_step = 0;
  for (const auto &field : out_fields) {
    out_step += _field_size(field);
  }

  sensor_msgs::msg::PointCloud2 downsampled;
  downsampled.header = msg->header;
  downsampled.fields = out_fields;
  downsampled.is_bigendian = msg->is_bigendian;
  downsampled.point_step = static_cast<uint32_t>(out_step);
  downsampled.height = 1;
  // Every point drops out of at most one voxel, so the input size is a safe
  // upper bound and one allocation covers the whole callback.
  downsampled.data.reserve(point_count * out_step);

  std::unordered_set<int64_t> seen_voxels;
  seen_voxels.reserve(point_count);

  for (std::size_t i = 0; i < point_count; ++i) {
    const std::size_t byte_offset = i * point_step;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    std::memcpy(&x, &msg->data[byte_offset + offsets.x], sizeof(float));
    std::memcpy(&y, &msg->data[byte_offset + offsets.y], sizeof(float));
    std::memcpy(&z, &msg->data[byte_offset + offsets.z], sizeof(float));
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    if (height_filter_transform.has_value()) {
      const tf2::Vector3 point_in_filter_frame =
          height_filter_transform.value() * tf2::Vector3(x, y, z);
      if (point_in_filter_frame.z() > _max_height_m) {
        continue;
      }
    }

    const int32_t vx = static_cast<int32_t>(std::floor(x / _voxel_size_m));
    const int32_t vy = static_cast<int32_t>(std::floor(y / _voxel_size_m));
    const int32_t vz = static_cast<int32_t>(std::floor(z / _voxel_size_m));
    if (!seen_voxels.insert(_voxel_key(vx, vy, vz)).second) {
      continue;
    }

    // Copied field by field rather than as one run of bytes, since the output
    // layout is a subset of the input's and is packed without its padding.
    for (const auto &field : out_fields) {
      const std::size_t size = _field_size(field);
      const std::size_t src = byte_offset + _source_offsets.at(field.name);
      downsampled.data.insert(downsampled.data.end(), msg->data.begin() + src,
                              msg->data.begin() + src + size);
    }
  }

  if (downsampled.data.empty()) {
    return;
  }

  downsampled.width = static_cast<uint32_t>(downsampled.data.size() / out_step);
  downsampled.row_step = static_cast<uint32_t>(downsampled.data.size());
  // Non-finite points were skipped above, so what's left is all valid.
  downsampled.is_dense = true;

  _publisher->publish(downsampled);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                       "Downsampled %zu -> %u points", point_count,
                       downsampled.width);
}

std::optional<VoxelDownsampler::position_offsets_t>
VoxelDownsampler::_resolve_position_offsets(
    const sensor_msgs::msg::PointCloud2 &msg) {
  using PointField = sensor_msgs::msg::PointField;

  const std::optional<std::size_t> x =
      find_field(msg, "x", PointField::FLOAT32);
  const std::optional<std::size_t> y =
      find_field(msg, "y", PointField::FLOAT32);
  const std::optional<std::size_t> z =
      find_field(msg, "z", PointField::FLOAT32);
  if (!x.has_value() || !y.has_value() || !z.has_value()) {
    return std::nullopt;
  }

  position_offsets_t offsets;
  offsets.x = x.value();
  offsets.y = y.value();
  offsets.z = z.value();

  // Each position read below is a fixed-width memcpy at one of these offsets,
  // so a cloud whose point_step doesn't actually cover the field it declares
  // would walk off the end of the buffer.
  const std::size_t required_end =
      std::max({offsets.x, offsets.y, offsets.z}) + sizeof(float);
  if (required_end > msg.point_step) {
    return std::nullopt;
  }

  return offsets;
}

std::optional<tf2::Transform>
VoxelDownsampler::_resolve_height_filter_transform(
    const std::string &cloud_frame_id) {
  if (!_height_filter_enabled) {
    return std::nullopt;
  }

  // Looked up fresh per cloud rather than cached: cheap relative to the
  // downsampling work below, and it keeps this correct across a TF frame
  // rename or listener restart without a node restart.
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = _tf_buffer->lookupTransform(
        _height_filter_frame, cloud_frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Height filter: could not look up %s -> %s (%s); "
                         "publishing this cloud unfiltered by height.",
                         cloud_frame_id.c_str(), _height_filter_frame.c_str(),
                         ex.what());
    return std::nullopt;
  }

  tf2::Transform transform;
  tf2::fromMsg(transform_stamped.transform, transform);
  return transform;
}

int64_t VoxelDownsampler::_voxel_key(int32_t vx, int32_t vy, int32_t vz) {
  const int64_t x_component =
      (static_cast<int64_t>(vx) + VOXEL_KEY_AXIS_OFFSET) & VOXEL_KEY_AXIS_MASK;
  const int64_t y_component =
      (static_cast<int64_t>(vy) + VOXEL_KEY_AXIS_OFFSET) & VOXEL_KEY_AXIS_MASK;
  const int64_t z_component =
      (static_cast<int64_t>(vz) + VOXEL_KEY_AXIS_OFFSET) & VOXEL_KEY_AXIS_MASK;
  return x_component | (y_component << VOXEL_KEY_BITS_PER_AXIS) |
         (z_component << (2 * VOXEL_KEY_BITS_PER_AXIS));
}

std::size_t
VoxelDownsampler::_field_size(const sensor_msgs::msg::PointField &field) {
  using PF = sensor_msgs::msg::PointField;
  std::size_t width = 0;
  switch (field.datatype) {
  case PF::INT8:
  case PF::UINT8:
    width = 1;
    break;
  case PF::INT16:
  case PF::UINT16:
    width = 2;
    break;
  case PF::INT32:
  case PF::UINT32:
  case PF::FLOAT32:
    width = 4;
    break;
  case PF::FLOAT64:
    width = 8;
    break;
  default:
    width = 0;
    break;
  }
  return width * std::max<std::size_t>(1, field.count);
}

std::vector<sensor_msgs::msg::PointField>
VoxelDownsampler::_select_output_fields(
    const sensor_msgs::msg::PointCloud2 &msg) {
  // The layout is fixed for a given publisher, so this is rebuilt only when the
  // input actually changes shape rather than on every cloud.
  const bool unchanged =
      _cached_input_fields.size() == msg.fields.size() &&
      std::equal(_cached_input_fields.begin(), _cached_input_fields.end(),
                 msg.fields.begin(),
                 [](const sensor_msgs::msg::PointField &a,
                    const sensor_msgs::msg::PointField &b) {
                   return a.name == b.name && a.offset == b.offset &&
                          a.datatype == b.datatype && a.count == b.count;
                 });
  if (unchanged) {
    return _cached_output_fields;
  }

  std::vector<sensor_msgs::msg::PointField> out;
  _source_offsets.clear();
  std::size_t offset = 0;
  // Driven by _keep_fields rather than by the input, so the output order is the
  // configured one whatever order the driver happens to publish in.
  const std::vector<std::string> wanted = _keep_fields.empty() ? [&] {
    std::vector<std::string> all;
    for (const auto &f : msg.fields) {
      all.push_back(f.name);
    }
    return all;
  }()
                                                               : _keep_fields;
  for (const auto &name : wanted) {
    for (const auto &field : msg.fields) {
      if (field.name != name) {
        continue;
      }
      const std::size_t size = _field_size(field);
      if (size == 0) {
        break;
      }
      sensor_msgs::msg::PointField copy = field;
      copy.offset = static_cast<uint32_t>(offset);
      out.push_back(copy);
      _source_offsets[name] = field.offset;
      offset += size;
      break;
    }
  }

  if (out.size() < wanted.size()) {
    RCLCPP_INFO(
        this->get_logger(),
        "Publishing %zu of the %zu requested fields; the input carries %zu.",
        out.size(), wanted.size(), msg.fields.size());
  }
  _cached_input_fields = msg.fields;
  _cached_output_fields = out;
  return out;
}

} // namespace sensors
