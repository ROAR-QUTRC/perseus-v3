/// @file voxel_downsampler.cpp
/// @brief Implementation of the voxel-grid downsampling node.

#include "sensors/voxel_downsampler/voxel_downsampler.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <sensor_msgs/msg/point_field.hpp>
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

  RCLCPP_INFO(this->get_logger(),
              "Voxel downsampler: %s -> %s (voxel size %.3f m)",
              input_topic.c_str(), output_topic.c_str(), _voxel_size_m);
}

void VoxelDownsampler::_load_parameters(std::string &input_topic_out,
                                        std::string &output_topic_out) {
  input_topic_out = this->declare_parameter("input_topic", DEFAULT_INPUT_TOPIC);
  output_topic_out =
      this->declare_parameter("output_topic", DEFAULT_OUTPUT_TOPIC);
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

  // Trust data.size() over width*height: a cloud that declares more points than
  // it carries would otherwise walk off the end of the buffer.
  const std::size_t point_step = msg->point_step;
  const std::size_t point_count =
      std::min(static_cast<std::size_t>(msg->width) * msg->height,
               msg->data.size() / point_step);

  sensor_msgs::msg::PointCloud2 downsampled;
  downsampled.header = msg->header;
  downsampled.fields = msg->fields;
  downsampled.is_bigendian = msg->is_bigendian;
  downsampled.point_step = msg->point_step;
  downsampled.height = 1;
  // Every point drops out of at most one voxel, so the input size is a safe
  // upper bound and one allocation covers the whole callback.
  downsampled.data.reserve(point_count * point_step);

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

    const int32_t vx = static_cast<int32_t>(std::floor(x / _voxel_size_m));
    const int32_t vy = static_cast<int32_t>(std::floor(y / _voxel_size_m));
    const int32_t vz = static_cast<int32_t>(std::floor(z / _voxel_size_m));
    if (!seen_voxels.insert(_voxel_key(vx, vy, vz)).second) {
      continue;
    }

    const auto point_begin = msg->data.begin() + byte_offset;
    downsampled.data.insert(downsampled.data.end(), point_begin,
                            point_begin + point_step);
  }

  if (downsampled.data.empty()) {
    return;
  }

  downsampled.width =
      static_cast<uint32_t>(downsampled.data.size() / point_step);
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

} // namespace sensors
