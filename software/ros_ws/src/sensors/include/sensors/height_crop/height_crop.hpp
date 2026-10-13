#pragma once

/// @file height_crop.hpp
/// @brief Drops point-cloud returns above a height in a reference frame -- the
/// ceiling, tent canopy or roof over the arena.

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

namespace sensors
{
    /// @brief ROS 2 node that removes every point more than `max_height_m` above
    ///        the origin of `reference_frame` (by default `odom`, whose z = 0 is the
    ///        ground the rover started on).
    ///
    /// Used on two streams: the live Livox scan, which then feeds the terrain
    /// costmaps and the base-station downlink, and BIEVR-LIO's /Laser_map, for the
    /// downlink only. The LIO itself keeps the raw scan -- the ceiling is good
    /// structure to register against, just not something to drive around or look at.
    ///
    /// Only the height test happens in the reference frame. The output keeps the
    /// input's header, frame and every field byte for byte (the Livox per-point
    /// timestamp, tag and line included), with the cropped points simply missing, so
    /// it is a drop-in replacement for the input topic. A cloud already stamped in the
    /// reference frame needs no transform; any other is transformed with TF at the
    /// cloud's own stamp, and dropped (not passed through uncropped) if that transform
    /// is not available.
    class HeightCrop : public rclcpp::Node
    {
    public:
        /// @brief Constructs the node, declaring parameters and setting up TF, the
        ///        subscription and the publisher.
        /// @param options Node options, supplied by main() or a component container.
        explicit HeightCrop(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
        /// @brief Default topic the raw point cloud is read from.
        static inline const std::string DEFAULT_INPUT_TOPIC = "/livox/lidar";
        /// @brief Default topic the cropped point cloud is published on.
        static inline const std::string DEFAULT_OUTPUT_TOPIC = "/livox/lidar/cropped";
        /// @brief Default frame the height is measured in.
        static inline const std::string DEFAULT_REFERENCE_FRAME = "odom";
        /// @brief Default height above the reference frame's origin to keep up to, in
        ///        metres.
        static constexpr double DEFAULT_MAX_HEIGHT_M = 1.5;
        /// @brief Default wait for the cloud's transform, in seconds.
        static constexpr double DEFAULT_TF_TIMEOUT_S = 0.05;

        /// @brief Crops an incoming cloud and republishes the result.
        /// @param msg Incoming point cloud.
        void _point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        std::string _reference_frame{DEFAULT_REFERENCE_FRAME};
        double _max_height_m{DEFAULT_MAX_HEIGHT_M};
        double _tf_timeout_s{DEFAULT_TF_TIMEOUT_S};

        std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
        std::unique_ptr<tf2_ros::TransformListener> _tf_listener;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscription;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher;
    };

}  // namespace sensors
