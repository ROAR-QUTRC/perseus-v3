/// @file voxel_downsampler.cpp
/// @brief Implementation of the voxel-grid downsampling + Draco compression
/// node.

#include "sensors/voxel_downsampler/voxel_downsampler.hpp"

#include <draco/compression/encode.h>
#include <draco/point_cloud/point_cloud_builder.h>

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

  _publisher = this->create_publisher<interfaces::msg::CompressedPointCloud>(
      output_topic, QOS_DEPTH);

  RCLCPP_INFO(
      this->get_logger(),
      "Voxel downsampler: %s -> %s (voxel size %.3f m, %d-bit Draco positions)",
      input_topic.c_str(), output_topic.c_str(), _voxel_size_m,
      _quantization_bits);
}

void VoxelDownsampler::_load_parameters(std::string &input_topic_out,
                                        std::string &output_topic_out) {
  input_topic_out = this->declare_parameter("input_topic", DEFAULT_INPUT_TOPIC);
  output_topic_out =
      this->declare_parameter("output_topic", DEFAULT_OUTPUT_TOPIC);
  _voxel_size_m = this->declare_parameter("voxel_size_m", DEFAULT_VOXEL_SIZE_M);
  _quantization_bits =
      this->declare_parameter("quantization_bits", DEFAULT_QUANTIZATION_BITS);

  // A non-positive voxel size would make the grid coordinate computation divide
  // by zero (or invert the grid), and Draco rejects a quantization outside
  // 1-30, so clamp both back to their defaults rather than publishing garbage.
  if (!(_voxel_size_m > 0.0)) {
    RCLCPP_WARN(this->get_logger(),
                "voxel_size_m must be positive, got %.3f -- falling back to "
                "%.3f m",
                _voxel_size_m, DEFAULT_VOXEL_SIZE_M);
    _voxel_size_m = DEFAULT_VOXEL_SIZE_M;
  }
  if (_quantization_bits < 1 || _quantization_bits > 30) {
    RCLCPP_WARN(this->get_logger(),
                "quantization_bits must be 1-30, got %d -- falling back to %d",
                _quantization_bits, DEFAULT_QUANTIZATION_BITS);
    _quantization_bits = DEFAULT_QUANTIZATION_BITS;
  }
}

void VoxelDownsampler::_point_cloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  const std::optional<field_offsets_t> resolved = _resolve_field_offsets(*msg);
  if (!resolved.has_value()) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Input cloud has no float32 x/y/z fields, ignoring.");
    return;
  }
  const field_offsets_t offsets = resolved.value();

  const std::size_t point_count =
      static_cast<std::size_t>(msg->width) * msg->height;

  std::unordered_set<int64_t> seen_voxels;
  seen_voxels.reserve(point_count);
  std::vector<point_t> kept_points;
  kept_points.reserve(point_count);

  for (std::size_t i = 0; i < point_count; ++i) {
    const point_t point = _read_point(*msg, offsets, i);
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }

    const int32_t vx =
        static_cast<int32_t>(std::floor(point.x / _voxel_size_m));
    const int32_t vy =
        static_cast<int32_t>(std::floor(point.y / _voxel_size_m));
    const int32_t vz =
        static_cast<int32_t>(std::floor(point.z / _voxel_size_m));
    if (seen_voxels.insert(_voxel_key(vx, vy, vz)).second) {
      kept_points.push_back(point);
    }
  }

  if (kept_points.empty()) {
    return;
  }

  const std::vector<uint8_t> draco_data = _encode_positions(kept_points);
  if (draco_data.empty()) {
    return;
  }

  interfaces::msg::CompressedPointCloud compressed_msg;
  compressed_msg.header = msg->header;
  compressed_msg.draco_data = draco_data;
  compressed_msg.quantization_bits = static_cast<uint8_t>(_quantization_bits);

  if (offsets.timestamp.has_value() && offsets.tag.has_value() &&
      offsets.line.has_value()) {
    compressed_msg.timestamps.reserve(kept_points.size());
    compressed_msg.tags.reserve(kept_points.size());
    compressed_msg.lines.reserve(kept_points.size());
    for (const point_t &point : kept_points) {
      compressed_msg.timestamps.push_back(point.timestamp);
      compressed_msg.tags.push_back(point.tag);
      compressed_msg.lines.push_back(point.line);
    }
  }

  _publisher->publish(compressed_msg);

  RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Downsampled %zu -> %zu points, Draco-compressed to %zu bytes",
      point_count, kept_points.size(), draco_data.size());
}

std::optional<VoxelDownsampler::field_offsets_t>
VoxelDownsampler::_resolve_field_offsets(
    const sensor_msgs::msg::PointCloud2 &msg) const {
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

  field_offsets_t offsets;
  offsets.x = x.value();
  offsets.y = y.value();
  offsets.z = z.value();
  offsets.timestamp = find_field(msg, "timestamp", PointField::FLOAT64);
  offsets.tag = find_field(msg, "tag", PointField::UINT8);
  offsets.line = find_field(msg, "line", PointField::UINT8);

  // Every read below is a fixed-width memcpy at one of these offsets, so a
  // cloud whose point_step doesn't actually cover the field it declares would
  // walk off the end of the buffer.
  const std::size_t none = 0;
  const std::size_t required_end = std::max(
      {offsets.x + sizeof(float), offsets.y + sizeof(float),
       offsets.z + sizeof(float),
       offsets.timestamp.has_value()
           ? offsets.timestamp.value() + sizeof(double)
           : none,
       offsets.tag.has_value() ? offsets.tag.value() + sizeof(uint8_t) : none,
       offsets.line.has_value() ? offsets.line.value() + sizeof(uint8_t)
                                : none});
  if (required_end > msg.point_step) {
    return std::nullopt;
  }

  return offsets;
}

VoxelDownsampler::point_t
VoxelDownsampler::_read_point(const sensor_msgs::msg::PointCloud2 &msg,
                              const field_offsets_t &offsets,
                              std::size_t point_index) {
  const std::size_t byte_offset = point_index * msg.point_step;
  point_t point;
  std::memcpy(&point.x, &msg.data[byte_offset + offsets.x], sizeof(float));
  std::memcpy(&point.y, &msg.data[byte_offset + offsets.y], sizeof(float));
  std::memcpy(&point.z, &msg.data[byte_offset + offsets.z], sizeof(float));
  if (offsets.timestamp.has_value()) {
    std::memcpy(&point.timestamp,
                &msg.data[byte_offset + offsets.timestamp.value()],
                sizeof(double));
  }
  if (offsets.tag.has_value()) {
    point.tag = msg.data[byte_offset + offsets.tag.value()];
  }
  if (offsets.line.has_value()) {
    point.line = msg.data[byte_offset + offsets.line.value()];
  }
  return point;
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

std::vector<uint8_t>
VoxelDownsampler::_encode_positions(const std::vector<point_t> &points) const {
  draco::PointCloudBuilder builder;
  builder.Start(points.size());
  const int position_attribute_id = builder.AddAttribute(
      draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);
  for (std::size_t i = 0; i < points.size(); ++i) {
    const float position[3] = {points[i].x, points[i].y, points[i].z};
    builder.SetAttributeValueForPoint(
        position_attribute_id, draco::PointIndex(static_cast<uint32_t>(i)),
        position);
  }
  // Deduplication would collapse points and silently break the index
  // correspondence with the timestamps/tags/lines arrays published alongside.
  constexpr bool deduplicate_points = false;
  const std::unique_ptr<draco::PointCloud> point_cloud =
      builder.Finalize(deduplicate_points);

  draco::Encoder encoder;
  encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION,
                                   _quantization_bits);
  encoder.SetEncodingMethod(draco::POINT_CLOUD_SEQUENTIAL_ENCODING);

  draco::EncoderBuffer buffer;
  const draco::Status status =
      encoder.EncodePointCloudToBuffer(*point_cloud, &buffer);
  if (!status.ok()) {
    RCLCPP_ERROR(this->get_logger(), "Draco encoding failed: %s",
                 status.error_msg());
    return {};
  }
  return std::vector<uint8_t>(buffer.data(), buffer.data() + buffer.size());
}

} // namespace sensors
