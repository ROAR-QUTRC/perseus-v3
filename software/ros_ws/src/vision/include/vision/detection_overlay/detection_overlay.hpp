#pragma once

/// @file detection_overlay.hpp
/// @brief Real-time detection overlay ROS 2 node.

#include <cstddef>
#include <cstdint>
#include <cv_bridge/cv_bridge.hpp>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "interfaces/msg/detection_array.hpp"

namespace vision {
/// @brief ROS 2 node that draws detections from several detectors onto one
/// image stream.
///
/// Subscribes to the shared camera source and to any number of DetectionArray
/// topics, and republishes the camera image annotated with every detector's
/// boxes.
///
/// The annotated stream is always published JPEG-compressed, on
/// `<output_image_topic>/compressed`. The overlay exists to be looked at by a
/// human, usually over the link off the vehicle, and a raw bgr8 frame costs
/// roughly an order of magnitude more bandwidth than the JPEG of it. The input
/// side is still switchable, since that is a property of whatever camera is
/// feeding this node.
///
/// The image path is deliberately never blocked waiting for detections. Each
/// frame is annotated with the most recent detections already received and
/// published immediately, so the output stream keeps the source frame rate
/// while the boxes themselves lag by however long inference takes. Detections
/// older than `max_detection_age_s` are dropped rather than left frozen on
/// screen.
class DetectionOverlay : public rclcpp::Node {
public:
  /// @brief Constructs the node, declaring parameters and setting up its
  /// topics.
  /// @param options Node options, supplied by the component container or by
  /// main().
  explicit DetectionOverlay(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  /// @brief Default topic that camera images are read from.
  static inline const std::string DEFAULT_INPUT_IMAGE_TOPIC =
      "/camera/camera/color/image_raw";
  /// @brief Default topic that the annotated image is published on.
  static inline const std::string DEFAULT_OUTPUT_IMAGE_TOPIC =
      "/vision/overlay/image";
  /// @brief Default detection topics subscribed to.
  static inline const std::vector<std::string> DEFAULT_DETECTION_TOPICS = {
      "/vision/aruco/detections", "/vision/cube/detections"};
  /// @brief Default age past which cached detections stop being drawn, in
  /// seconds.
  static constexpr double DEFAULT_MAX_DETECTION_AGE_S = 1.0;
  /// @brief Default for whether the camera source is compressed.
  static constexpr bool DEFAULT_IS_COMPRESSED_INPUT = false;
  /// @brief Default for whether detection staleness is drawn on the image.
  static constexpr bool DEFAULT_SHOULD_SHOW_STALENESS = false;

  /// @brief Annotates and republishes a raw camera image.
  /// @param msg Incoming raw image message.
  void _image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  /// @brief Annotates and republishes a compressed camera image.
  /// @param msg Incoming compressed image message.
  void _compressed_image_callback(
      const sensor_msgs::msg::CompressedImage::SharedPtr msg);

  /// @brief JPEG-encodes an annotated frame and publishes it.
  ///
  /// The single publish path shared by both input callbacks.
  ///
  /// @param header Header of the source image, republished unchanged so the
  /// annotated frame keeps the capture timestamp and frame_id.
  /// @param frame Annotated image to encode.
  void _publish_overlay(const std_msgs::msg::Header &header,
                        const cv::Mat &frame);

  /// @brief Caches the latest detections from one detector.
  /// @param topic Topic the detections arrived on, used as the cache key.
  /// @param msg Incoming detections.
  void
  _detections_callback(const std::string &topic,
                       const interfaces::msg::DetectionArray::SharedPtr msg);

  /// @brief Draws every cached detection that is still fresh onto a frame.
  /// @param frame Image to annotate. Modified in place.
  /// @param now Current time, used to age out stale detections.
  /// @return Number of detections drawn.
  std::size_t _draw_cached_detections(cv::Mat &frame, const rclcpp::Time &now);

  // Parameters
  std::string _input_image_topic{DEFAULT_INPUT_IMAGE_TOPIC};
  std::string _output_image_topic{DEFAULT_OUTPUT_IMAGE_TOPIC};
  std::vector<std::string> _detection_topics{DEFAULT_DETECTION_TOPICS};
  double _max_detection_age_s{DEFAULT_MAX_DETECTION_AGE_S};
  bool _is_compressed_input{DEFAULT_IS_COMPRESSED_INPUT};
  bool _should_show_staleness{DEFAULT_SHOULD_SHOW_STALENESS};

  // ROS IO
  // Exactly one of the two subscriptions is created, depending on
  // `compressed_input`; the publisher is always the compressed one.
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_subscription;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
      _compressed_image_subscription;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
      _compressed_image_publisher;
  std::vector<rclcpp::Subscription<interfaces::msg::DetectionArray>::SharedPtr>
      _detection_subscriptions;

  // Latest detections per source topic
  std::mutex _detections_mutex;
  std::unordered_map<std::string, interfaces::msg::DetectionArray>
      _latest_detections;
};

} // namespace vision
