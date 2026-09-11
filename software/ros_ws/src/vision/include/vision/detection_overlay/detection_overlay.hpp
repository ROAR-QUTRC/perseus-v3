#pragma once

/// @file detection_overlay.hpp
/// @brief Real-time detection overlay ROS 2 node.

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cstddef>
#include <cstdint>
#include <cv_bridge/cv_bridge.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "interfaces/msg/detection_array.hpp"

namespace vision
{
    /// @brief ROS 2 node that draws detections from several detectors onto one
    /// image stream.
    ///
    /// Subscribes to the shared camera source and to any number of DetectionArray
    /// topics, and republishes the camera image annotated with every detector's
    /// boxes.
    ///
    /// The image path is deliberately never blocked waiting for detections. Each
    /// frame is annotated with the most recent detections already received and
    /// published immediately, so the output stream keeps the source frame rate
    /// while the boxes themselves lag by however long inference takes. Detections
    /// older than `max_detection_age_s` are dropped rather than left frozen on
    /// screen.
    class DetectionOverlay : public rclcpp::Node
    {
    public:
        /// @brief Constructs the node, declaring parameters and setting up its
        /// topics.
        /// @param options Node options, supplied by the component container or by
        /// main().
        explicit DetectionOverlay(
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

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
        static constexpr bool DEFAULT_INPUT_COMPRESSED = false;
        /// @brief Default for whether the published overlay is compressed.
        static constexpr bool DEFAULT_OUTPUT_COMPRESSED = true;
        /// @brief Default for whether detection staleness is drawn on the image.
        static constexpr bool DEFAULT_SHOULD_SHOW_STALENESS = false;
        /// @brief Default topic that the nav2 global plan is read from.
        static inline const std::string DEFAULT_PATH_TOPIC = "/plan";
        /// @brief Default topic that camera calibration is read from.
        static inline const std::string DEFAULT_CAMERA_INFO_TOPIC =
            "/camera/camera/color/camera_info";
        /// @brief Default for whether the plan is drawn on the image.
        static constexpr bool DEFAULT_SHOULD_SHOW_PATH = true;

        /// @brief Annotates and republishes a raw camera image.
        /// @param msg Incoming raw image message.
        void _image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

        /// @brief Annotates and republishes a compressed camera image.
        /// @param msg Incoming compressed image message.
        void _compressed_image_callback(
            const sensor_msgs::msg::CompressedImage::SharedPtr msg);

        /// @brief Publishes an annotated frame, encoding it to match whichever
        /// output format is configured.
        /// @param frame Annotated image to publish.
        /// @param header Header to stamp the outgoing message with.
        void _publish_frame(const cv::Mat& frame, const std_msgs::msg::Header& header);

        /// @brief Caches the latest nav2 global plan.
        /// @param msg Incoming plan.
        void _path_callback(const nav_msgs::msg::Path::SharedPtr msg);

        /// @brief Caches the latest camera calibration.
        /// @param msg Incoming camera calibration message.
        void _camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

        /// @brief Projects the cached plan into the image and draws it in blue.
        ///
        /// Each plan pose is transformed from the plan's frame into the camera
        /// optical frame named by the cached camera_info, then projected with the
        /// pinhole model built from that calibration. Points behind the camera break
        /// the drawn line rather than being connected across the discontinuity.
        /// @param frame Image to draw onto. Modified in place.
        void _draw_path(cv::Mat& frame);

        /// @brief Caches the latest detections from one detector.
        /// @param topic Topic the detections arrived on, used as the cache key.
        /// @param msg Incoming detections.
        void
        _detections_callback(const std::string& topic,
                             const interfaces::msg::DetectionArray::SharedPtr msg);

        /// @brief Draws every cached detection that is still fresh onto a frame.
        /// @param frame Image to annotate. Modified in place.
        /// @param now Current time, used to age out stale detections.
        /// @return Number of detections drawn.
        std::size_t _draw_cached_detections(cv::Mat& frame, const rclcpp::Time& now);

        // Parameters
        std::string _input_image_topic{DEFAULT_INPUT_IMAGE_TOPIC};
        std::string _output_image_topic{DEFAULT_OUTPUT_IMAGE_TOPIC};
        std::vector<std::string> _detection_topics{DEFAULT_DETECTION_TOPICS};
        double _max_detection_age_s{DEFAULT_MAX_DETECTION_AGE_S};
        bool _input_compressed{DEFAULT_INPUT_COMPRESSED};
        bool _output_compressed{DEFAULT_OUTPUT_COMPRESSED};
        bool _should_show_staleness{DEFAULT_SHOULD_SHOW_STALENESS};
        std::string _path_topic{DEFAULT_PATH_TOPIC};
        std::string _camera_info_topic{DEFAULT_CAMERA_INFO_TOPIC};
        bool _should_show_path{DEFAULT_SHOULD_SHOW_PATH};

        // ROS IO
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_subscription;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_publisher;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
            _compressed_image_subscription;
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr
            _compressed_image_publisher;
        std::vector<rclcpp::Subscription<interfaces::msg::DetectionArray>::SharedPtr>
            _detection_subscriptions;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_subscription;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
            _camera_info_subscription;

        // Latest detections per source topic
        std::mutex _detections_mutex;
        std::unordered_map<std::string, interfaces::msg::DetectionArray>
            _latest_detections;

        // Latest plan
        std::mutex _path_mutex;
        nav_msgs::msg::Path _latest_path;

        // Latest camera calibration
        std::mutex _camera_info_mutex;
        sensor_msgs::msg::CameraInfo _latest_camera_info;
        bool _has_camera_info{false};

        // TF
        std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> _tf_listener;
    };

}  // namespace vision
