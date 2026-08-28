#pragma once

/// @file voxel_downsampler.hpp
/// @brief Thins a point cloud down to at most one point per voxel cell.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace sensors {
/// @brief ROS 2 node that republishes a point cloud keeping at most one point
///        per voxel cell.
///
/// Surviving points are copied through byte for byte, so whatever fields the
/// input carries (intensity, timestamp, tag, line, ...) come out unchanged --
/// only the point count drops. Compression of the thinned cloud is left to
/// point_cloud_transport's republish node, which sits downstream of this one.
class VoxelDownsampler : public rclcpp::Node {
public:
  /// @brief Constructs the node, declaring parameters and setting up the
  ///        subscription and publisher.
  /// @param options Node options, supplied by main().
  explicit VoxelDownsampler(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  /// @brief Default topic that the raw point cloud is read from.
  static inline const std::string DEFAULT_INPUT_TOPIC = "/livox/lidar";
  /// @brief Default topic that the thinned point cloud is published on.
  static inline const std::string DEFAULT_OUTPUT_TOPIC =
      "/livox/lidar/downsampled";

  /// @brief Default voxel cell size, in metres.
  static constexpr double DEFAULT_VOXEL_SIZE_M = 0.1;

  /// @brief Byte offsets of the position fields within a point.
  struct position_offsets_t {
    std::size_t x{0};
    std::size_t y{0};
    std::size_t z{0};
  };

  /// @brief Declares every parameter and copies the topic names into the
  ///        matching out-parameters.
  /// @param input_topic_out Receives the resolved input point cloud topic.
  /// @param output_topic_out Receives the resolved output point cloud topic.
  void _load_parameters(std::string &input_topic_out,
                        std::string &output_topic_out);

  /// @brief Downsamples an incoming cloud and republishes it.
  /// @param msg Incoming point cloud.
  void
  _point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /// @brief Resolves the byte offsets of the float32 x/y/z fields in a cloud.
  /// @param msg Cloud to resolve field offsets against.
  /// @return The resolved offsets, or nullopt if the cloud lacks float32
  ///         x/y/z or declares them beyond the end of a point.
  static std::optional<position_offsets_t>
  _resolve_position_offsets(const sensor_msgs::msg::PointCloud2 &msg);

  /// @brief Bytes one field occupies, from its datatype and count.
  static std::size_t _field_size(const sensor_msgs::msg::PointField &field);

  /// @brief The output layout: the fields named by the keep_fields parameter
  /// that the input actually has, repacked from offset zero with no padding.
  /// Also records where each one sits in the input, in _source_offsets, for the
  /// copy.
  std::vector<sensor_msgs::msg::PointField>
  _select_output_fields(const sensor_msgs::msg::PointCloud2 &msg);

  /// @brief Packs a voxel's integer coordinates into a single hashable key.
  /// @param vx Voxel coordinate along x, in units of the voxel size.
  /// @param vy Voxel coordinate along y, in units of the voxel size.
  /// @param vz Voxel coordinate along z, in units of the voxel size.
  static int64_t _voxel_key(int32_t vx, int32_t vy, int32_t vz);

  double _voxel_size_m{DEFAULT_VOXEL_SIZE_M};

  /// @brief Field names to carry through, in order. Anything else the input has
  /// is dropped. Empty keeps every field, which is the escape hatch for a
  /// consumer that genuinely wants the per-return metadata and is not going
  /// through Draco.
  std::vector<std::string> _keep_fields;
  /// @brief Where each kept field sits in the *input* point, rebuilt whenever
  /// the input layout changes.
  std::unordered_map<std::string, std::size_t> _source_offsets;
  /// @brief The layout _source_offsets was built for, so it is only rebuilt on
  /// a change.
  std::vector<sensor_msgs::msg::PointField> _cached_input_fields;
  std::vector<sensor_msgs::msg::PointField> _cached_output_fields;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      _point_cloud_subscription;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher;
};

} // namespace sensors
