#pragma once

/// @file orb_slam_odometry.hpp
/// @brief Stereo visual odometry from ORB-SLAM3, for the EKF's second pose source.

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sophus/se3.hpp>
#include <string>
#include <thread>

#include "interfaces/msg/orb_slam_odometry_info.hpp"
#include "vision/common/odometry_publisher.hpp"

namespace ORB_SLAM3
{
    class System;
}

namespace vision
{
    /// @brief ROS 2 node running ORB-SLAM3 on a rectified stereo pair and publishing the
    ///        camera's motion as odometry.
    ///
    /// ORB-SLAM3 tracks against a local map of keyframes rather than only the previous
    /// frame, so it drifts less than frame-to-frame VO. What it reports, though, is a pose
    /// in whatever map is current, and that pose can jump: on relocalisation, when a new
    /// map is started after tracking is lost, or when a merge or global bundle adjustment
    /// moves the map. None of that may reach the EKF. So this node publishes only the
    /// *step* between consecutive tracked poses in the same map, through the same
    /// OdometryPublisher libviso2 used, and replaces any step across a discontinuity with
    /// an identity step carrying the lost-tracking covariance. The published odometry is
    /// continuous by construction.
    ///
    /// Covariance follows tracking quality: the configured variances hold while at least
    /// `reference_map_points` map points are tracked, and scale up by
    /// reference_map_points / tracked as fewer are, to at most `max_covariance_scale`
    /// times. Below `min_map_points`, or when not tracking, the step is published with
    /// the lost-tracking covariance instead, which the EKF effectively ignores.
    ///
    /// Loading the vocabulary and building ORB-SLAM3 takes seconds, and tracking a frame
    /// tens of milliseconds, so both happen on a worker thread: the image callback only
    /// hands over the newest synchronised pair, and a pair that arrives while the worker
    /// is busy replaces the waiting one. Loaded into the camera driver's container, that
    /// keeps the driver's own thread free.
    ///
    /// ORB-SLAM3 is built headless (no viewer) with loop closing off by default: as an
    /// odometry source it has no use for loop closure, which only adds CPU spikes and map
    /// corrections this node would have to discard anyway.
    class OrbSlamOdometry : public rclcpp::Node
    {
    public:
        /// @brief Declares parameters and sets up the synchronised subscribers, the
        ///        publishers and the worker thread.
        /// @param options Node options, supplied by the component container or by main().
        explicit OrbSlamOdometry(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        /// @brief Stops the worker and shuts ORB-SLAM3 down.
        ~OrbSlamOdometry() override;

    private:
        using ApproximateSyncPolicy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image,
            sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo>;
        using ApproximateSynchronizer = message_filters::Synchronizer<ApproximateSyncPolicy>;

        /// @brief One synchronised stereo pair waiting for the worker.
        struct stereo_frame_t
        {
            cv_bridge::CvImageConstPtr left;
            cv_bridge::CvImageConstPtr right;
            rclcpp::Time stamp;
        };

        /// @brief Camera parameters ORB-SLAM3 is configured from, read off camera_info.
        struct stereo_calibration_t
        {
            double fx{0.0};
            double fy{0.0};
            double cx{0.0};
            double cy{0.0};
            double baseline_m{0.0};
            uint32_t width{0};
            uint32_t height{0};
            std::string frame_id;
            std::string right_frame_id;
        };

        /// @brief Declares every parameter and returns the odometry publisher's part.
        odometry_publisher_config_t _load_parameters();

        /// @brief Hands the newest synchronised pair to the worker, dropping any older one
        ///        it has not started on.
        void _stereo_callback(const sensor_msgs::msg::Image::ConstSharedPtr& left_image,
                              const sensor_msgs::msg::Image::ConstSharedPtr& right_image,
                              const sensor_msgs::msg::CameraInfo::ConstSharedPtr& left_info,
                              const sensor_msgs::msg::CameraInfo::ConstSharedPtr& right_info);

        /// @brief The worker: builds ORB-SLAM3 once calibration is known, then tracks
        ///        each handed-over pair.
        void _worker();

        /// @brief Settles the stereo baseline before ORB-SLAM3 starts: the `baseline_m`
        ///        override if set, else camera_info's, unless TF between the two camera
        ///        frames disagrees with it by more than 10%, in which case TF's.
        ///
        /// The baseline is the scale of every stereo depth, so an error in it is the same
        /// error in every translation (rotation is unaffected). perseus_simulation's
        /// right camera_info has reported 36 mm against the 95 mm its cameras are
        /// rendered apart, which made straight-line motion read at 0.38x.
        /// @return The baseline to use, in metres; 0 if there is none.
        double _resolve_baseline(const stereo_calibration_t& calibration);

        /// @brief Where the odometry starts: the rover's current odom -> base_link pose
        ///        from TF if `initial_pose_from_tf` and it is available, else identity.
        ///
        /// Starting at identity put the odometry's origin, heading and level wherever
        /// the rover happened to be when ORB-SLAM3 finished loading, so its path ran off
        /// at an angle to everything else in odom and, if the rover was on a slope,
        /// tilted -- part of every forward step read as climbing or descending. Seeded
        /// from the EKF's pose it overlays the other sources and inherits the IMU's level.
        tf2::Transform _initial_pose();

        /// @brief Writes ORB-SLAM3's settings file for this camera and these parameters.
        /// @return The file's path, or nullopt if it could not be written.
        std::optional<std::string> _write_settings(const stereo_calibration_t& calibration) const;

        /// @brief Tracks one pair and publishes the resulting step and diagnostics.
        void _track(const stereo_frame_t& frame);

        /// @brief Publishes an identity step with the lost-tracking covariance.
        void _publish_no_step(const rclcpp::Time& stamp);

        /// @brief The configured covariance (diagonal: three linear then three angular
        ///        variances) multiplied by `scale`.
        static std::array<double, 36> _diagonal_covariance(double linear, double angular, double scale);

        // ---- parameters ----
        std::string _vocabulary_path;
        int _n_features{1000};
        double _scale_factor{1.2};
        int _n_levels{8};
        int _ini_th_fast{20};
        int _min_th_fast{7};
        double _th_depth{40.0};
        int _camera_fps{30};
        double _baseline_override_m{0.0};
        bool _initial_pose_from_tf{true};
        std::string _odom_frame_id;
        std::string _base_link_frame_id;
        bool _loop_closing{false};
        double _processing_frequency_hz{15.0};
        double _max_step_translation_m{0.25};
        double _max_step_rotation_rad{0.35};
        double _position_variance{0.1};
        double _orientation_variance{0.17};
        double _linear_velocity_variance{0.002};
        double _angular_velocity_variance{0.09};
        int _reference_map_points{150};
        int _min_map_points{30};
        double _max_covariance_scale{20.0};

        // ---- ROS ----
        image_transport::SubscriberFilter _left_image_subscriber;
        image_transport::SubscriberFilter _right_image_subscriber;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> _left_camera_info_subscriber;
        message_filters::Subscriber<sensor_msgs::msg::CameraInfo> _right_camera_info_subscriber;
        std::shared_ptr<ApproximateSynchronizer> _synchronizer;
        std::unique_ptr<OdometryPublisher> _odometry_publisher;
        std::unique_ptr<tf2_ros::Buffer> _tf_buffer;
        std::unique_ptr<tf2_ros::TransformListener> _tf_listener;
        rclcpp::Publisher<interfaces::msg::OrbSlamOdometryInfo>::SharedPtr _info_publisher;

        // ---- handover to the worker ----
        std::mutex _frame_mutex;
        std::condition_variable _frame_ready;
        std::optional<stereo_frame_t> _pending_frame;
        std::optional<stereo_calibration_t> _calibration;
        rclcpp::Time _last_accepted_stamp{0, 0, RCL_ROS_TIME};
        std::atomic<bool> _stopping{false};
        std::thread _worker_thread;

        // ---- worker-only state ----
        std::unique_ptr<ORB_SLAM3::System> _slam;
        std::string _settings_path;
        /// Camera pose in ORB-SLAM3's current map at the last frame tracked OK, or empty
        /// when the next tracked frame starts a new run of steps.
        std::optional<Sophus::SE3f> _last_camera_pose;
    };

}  // namespace vision
