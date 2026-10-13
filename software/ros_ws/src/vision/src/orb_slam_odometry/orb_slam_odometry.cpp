/// @file orb_slam_odometry.cpp
/// @brief Implementation and component registration for OrbSlamOdometry.

#include "vision/orb_slam_odometry/orb_slam_odometry.hpp"

#include <System.h>
#include <Tracking.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace vision
{
    namespace
    {
        constexpr int QOS_DEPTH = 10;
        constexpr int IMAGE_QUEUE_DEPTH = 5;
        constexpr int SYNC_QUEUE_SIZE = 10;
        constexpr int64_t WARN_THROTTLE_MS = 10000;

        /// @brief Variance reported for a step that carries no information: huge, so the
        ///        EKF gives it no weight. Same value libviso2's node used.
        constexpr double LOST_TRACKING_VARIANCE = 9999.0;

        /// @brief Baseline (m) of a rectified stereo pair, from the right camera's P matrix.
        ///
        /// Per REP 104 the right projection carries P[3] = -fx * baseline, the computation
        /// image_geometry::StereoCameraModel::baseline() performs.
        double baseline_from_right_projection(const sensor_msgs::msg::CameraInfo& right_info)
        {
            const double fx = right_info.p[0];
            return fx != 0.0 ? -right_info.p[3] / fx : 0.0;
        }

        tf2::Transform to_tf(const Sophus::SE3f& pose)
        {
            const Eigen::Quaternionf q = pose.unit_quaternion();
            const Eigen::Vector3f t = pose.translation();
            return tf2::Transform(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()), tf2::Vector3(t.x(), t.y(), t.z()));
        }
    }  // namespace

    OrbSlamOdometry::OrbSlamOdometry(const rclcpp::NodeOptions& options)
        : Node("orb_slam_odometry", options)
    {
        const odometry_publisher_config_t odometry_config = _load_parameters();
        _odometry_publisher = std::make_unique<OdometryPublisher>(*this, odometry_config);
        _tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        _tf_listener = std::make_unique<tf2_ros::TransformListener>(*_tf_buffer);
        _info_publisher = create_publisher<interfaces::msg::OrbSlamOdometryInfo>(
            declare_parameter("output_info_topic", std::string("/vision/orb_slam_odometry/info")), QOS_DEPTH);

        rmw_qos_profile_t image_qos = rmw_qos_profile_sensor_data;
        image_qos.depth = IMAGE_QUEUE_DEPTH;
        _left_image_subscriber.subscribe(
            this, declare_parameter("left_image_topic", std::string("/camera/camera/infra1/image_rect_raw")),
            "raw", image_qos);
        _right_image_subscriber.subscribe(
            this, declare_parameter("right_image_topic", std::string("/camera/camera/infra2/image_rect_raw")),
            "raw", image_qos);
        _left_camera_info_subscriber.subscribe(
            this, declare_parameter("left_camera_info_topic", std::string("/camera/camera/infra1/camera_info")),
            image_qos);
        _right_camera_info_subscriber.subscribe(
            this, declare_parameter("right_camera_info_topic", std::string("/camera/camera/infra2/camera_info")),
            image_qos);
        _synchronizer = std::make_shared<ApproximateSynchronizer>(
            ApproximateSyncPolicy(SYNC_QUEUE_SIZE), _left_image_subscriber, _right_image_subscriber,
            _left_camera_info_subscriber, _right_camera_info_subscriber);
        _synchronizer->registerCallback(std::bind(&OrbSlamOdometry::_stereo_callback, this, std::placeholders::_1,
                                                  std::placeholders::_2, std::placeholders::_3,
                                                  std::placeholders::_4));

        _worker_thread = std::thread(&OrbSlamOdometry::_worker, this);
        RCLCPP_INFO(get_logger(), "OrbSlamOdometry waiting for the first stereo pair to configure ORB-SLAM3");
    }

    OrbSlamOdometry::~OrbSlamOdometry()
    {
        _stopping = true;
        _frame_ready.notify_all();
        if (_worker_thread.joinable())
        {
            _worker_thread.join();
        }
        if (_slam)
        {
            _slam->Shutdown();
        }
        if (!_settings_path.empty())
        {
            std::error_code ignored;
            std::filesystem::remove(_settings_path, ignored);
        }
    }

    odometry_publisher_config_t OrbSlamOdometry::_load_parameters()
    {
        // Empty: the vocabulary installed with the ORB_SLAM3 package this was built against.
        _vocabulary_path = declare_parameter("vocabulary_path", std::string(""));
        if (_vocabulary_path.empty())
        {
            _vocabulary_path = ORB_SLAM3_DEFAULT_VOCABULARY;
        }
        _n_features = declare_parameter("orb.n_features", _n_features);
        _scale_factor = declare_parameter("orb.scale_factor", _scale_factor);
        _n_levels = declare_parameter("orb.n_levels", _n_levels);
        _ini_th_fast = declare_parameter("orb.ini_th_fast", _ini_th_fast);
        _min_th_fast = declare_parameter("orb.min_th_fast", _min_th_fast);
        _th_depth = declare_parameter("th_depth", _th_depth);
        _camera_fps = declare_parameter("camera_fps", _camera_fps);
        _baseline_override_m = declare_parameter("baseline_m", _baseline_override_m);
        _loop_closing = declare_parameter("loop_closing", _loop_closing);
        _processing_frequency_hz = declare_parameter("processing_frequency_hz", _processing_frequency_hz);
        _max_step_translation_m = declare_parameter("max_step_translation_m", _max_step_translation_m);
        _max_step_rotation_rad = declare_parameter("max_step_rotation_rad", _max_step_rotation_rad);
        _position_variance = declare_parameter("covariance.position_variance", _position_variance);
        _orientation_variance = declare_parameter("covariance.orientation_variance", _orientation_variance);
        _linear_velocity_variance =
            declare_parameter("covariance.linear_velocity_variance", _linear_velocity_variance);
        _angular_velocity_variance =
            declare_parameter("covariance.angular_velocity_variance", _angular_velocity_variance);
        _reference_map_points = declare_parameter("covariance.reference_map_points", _reference_map_points);
        _min_map_points = declare_parameter("covariance.min_map_points", _min_map_points);
        _max_covariance_scale = declare_parameter("covariance.max_scale", _max_covariance_scale);

        odometry_publisher_config_t config;
        config.odom_frame_id = declare_parameter("odom_frame_id", config.odom_frame_id);
        config.base_link_frame_id = declare_parameter("base_link_frame_id", config.base_link_frame_id);
        _odom_frame_id = config.odom_frame_id;
        _base_link_frame_id = config.base_link_frame_id;
        _initial_pose_from_tf = declare_parameter("initial_pose_from_tf", _initial_pose_from_tf);
        config.sensor_frame_id = declare_parameter("sensor_frame_id", std::string("camera_infra1_optical_frame"));
        config.odometry_topic = declare_parameter("output_odometry_topic", config.odometry_topic);
        config.pose_topic = declare_parameter("output_pose_topic", config.pose_topic);
        config.should_publish_tf = declare_parameter("publish_tf", false);
        config.should_invert_tf = declare_parameter("invert_tf", false);
        config.is_initial_pose_in_camera_frame = false;
        return config;
    }

    void OrbSlamOdometry::_stereo_callback(const sensor_msgs::msg::Image::ConstSharedPtr& left_image,
                                           const sensor_msgs::msg::Image::ConstSharedPtr& right_image,
                                           const sensor_msgs::msg::CameraInfo::ConstSharedPtr& left_info,
                                           const sensor_msgs::msg::CameraInfo::ConstSharedPtr& right_info)
    {
        const rclcpp::Time stamp(left_image->header.stamp, RCL_ROS_TIME);
        {
            std::lock_guard<std::mutex> lock(_frame_mutex);
            // Rate cap: on the Orange Pi the full 30 Hz is more than ORB-SLAM3 keeps up
            // with alongside the LIO and nav, so frames are thinned before conversion.
            if (_processing_frequency_hz > 0.0 && _last_accepted_stamp.nanoseconds() > 0 &&
                (stamp - _last_accepted_stamp).seconds() < 0.95 / _processing_frequency_hz &&
                stamp >= _last_accepted_stamp)
            {
                return;
            }
            _last_accepted_stamp = stamp;
            if (!_calibration)
            {
                stereo_calibration_t calibration;
                // Rectified images: the left P holds the common intrinsics.
                calibration.fx = left_info->p[0];
                calibration.fy = left_info->p[5];
                calibration.cx = left_info->p[2];
                calibration.cy = left_info->p[6];
                calibration.baseline_m = baseline_from_right_projection(*right_info);
                calibration.width = left_info->width;
                calibration.height = left_info->height;
                calibration.frame_id = left_info->header.frame_id;
                calibration.right_frame_id = right_info->header.frame_id;
                _calibration = calibration;
            }
        }

        stereo_frame_t frame;
        try
        {
            frame.left = cv_bridge::toCvShare(left_image, sensor_msgs::image_encodings::MONO8);
            frame.right = cv_bridge::toCvShare(right_image, sensor_msgs::image_encodings::MONO8);
        }
        catch (const cv_bridge::Exception& error)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), WARN_THROTTLE_MS, "cv_bridge: %s", error.what());
            return;
        }
        frame.stamp = stamp;
        {
            std::lock_guard<std::mutex> lock(_frame_mutex);
            _pending_frame = std::move(frame);
        }
        _frame_ready.notify_one();
    }

    void OrbSlamOdometry::_worker()
    {
        while (!_stopping)
        {
            std::optional<stereo_frame_t> frame;
            std::optional<stereo_calibration_t> calibration;
            {
                std::unique_lock<std::mutex> lock(_frame_mutex);
                _frame_ready.wait(lock, [this]
                                  { return _stopping || _pending_frame.has_value(); });
                if (_stopping)
                {
                    return;
                }
                frame.swap(_pending_frame);
                calibration = _calibration;
            }

            if (!_slam)
            {
                if (calibration)
                {
                    calibration->baseline_m = _resolve_baseline(*calibration);
                }
                if (!calibration || calibration->fx <= 0.0 || calibration->baseline_m <= 0.0)
                {
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), WARN_THROTTLE_MS,
                                          "Unusable stereo camera_info (fx %.1f, baseline %.4f m); waiting",
                                          calibration ? calibration->fx : 0.0,
                                          calibration ? calibration->baseline_m : 0.0);
                    continue;
                }
                // ORB-SLAM3 calls exit() on a missing vocabulary or settings file, which
                // would take the whole camera container down with it, so check first.
                if (!std::filesystem::exists(_vocabulary_path))
                {
                    RCLCPP_FATAL(get_logger(), "ORB vocabulary not found at '%s'; odometry disabled",
                                 _vocabulary_path.c_str());
                    return;
                }
                const auto settings_path = _write_settings(*calibration);
                if (!settings_path)
                {
                    return;
                }
                _settings_path = *settings_path;
                if (!calibration->frame_id.empty())
                {
                    _odometry_publisher->set_sensor_frame_id(calibration->frame_id);
                }
                RCLCPP_INFO(get_logger(),
                            "Starting ORB-SLAM3 (stereo, %ux%u, fx %.1f, baseline %.4f m, %d features, "
                            "loop closing %s); loading the vocabulary takes a while",
                            calibration->width, calibration->height, calibration->fx,
                            calibration->baseline_m, _n_features, _loop_closing ? "on" : "off");
                _slam = std::make_unique<ORB_SLAM3::System>(_vocabulary_path, _settings_path,
                                                            ORB_SLAM3::System::STEREO, false);
                RCLCPP_INFO(get_logger(), "ORB-SLAM3 ready");
                _odometry_publisher->set_initial_pose(_initial_pose());
                continue;  // this frame is stale after the load; track the next one
            }

            _track(*frame);
        }
    }

    tf2::Transform OrbSlamOdometry::_initial_pose()
    {
        if (!_initial_pose_from_tf)
        {
            return tf2::Transform::getIdentity();
        }
        try
        {
            const auto transform = _tf_buffer->lookupTransform(_odom_frame_id, _base_link_frame_id, tf2::TimePointZero,
                                                               tf2::durationFromSec(2.0));
            const auto& t = transform.transform.translation;
            const auto& q = transform.transform.rotation;
            const tf2::Transform pose(tf2::Quaternion(q.x, q.y, q.z, q.w), tf2::Vector3(t.x, t.y, t.z));
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            pose.getBasis().getRPY(roll, pitch, yaw);
            RCLCPP_INFO(get_logger(),
                        "Odometry starts at the current %s -> %s pose: (%.2f, %.2f, %.2f) m, roll %.1f pitch %.1f "
                        "yaw %.1f deg",
                        _odom_frame_id.c_str(), _base_link_frame_id.c_str(), t.x, t.y, t.z, roll * 180.0 / M_PI,
                        pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
            return pose;
        }
        catch (const tf2::TransformException& error)
        {
            RCLCPP_WARN(get_logger(), "No %s -> %s transform yet (%s); odometry starts at identity",
                        _odom_frame_id.c_str(), _base_link_frame_id.c_str(), error.what());
            return tf2::Transform::getIdentity();
        }
    }

    double OrbSlamOdometry::_resolve_baseline(const stereo_calibration_t& calibration)
    {
        if (_baseline_override_m > 0.0)
        {
            RCLCPP_INFO(get_logger(), "Stereo baseline %.4f m from the baseline_m parameter (camera_info: %.4f m)",
                        _baseline_override_m, calibration.baseline_m);
            return _baseline_override_m;
        }
        if (calibration.frame_id.empty() || calibration.right_frame_id.empty())
        {
            return calibration.baseline_m;
        }
        double tf_baseline = 0.0;
        try
        {
            const auto transform = _tf_buffer->lookupTransform(calibration.frame_id, calibration.right_frame_id,
                                                               tf2::TimePointZero, tf2::durationFromSec(2.0));
            const auto& t = transform.transform.translation;
            tf_baseline = std::sqrt((t.x * t.x) + (t.y * t.y) + (t.z * t.z));
        }
        catch (const tf2::TransformException& error)
        {
            RCLCPP_WARN(get_logger(), "No %s -> %s transform to check the baseline against (%s); using camera_info's %.4f m",
                        calibration.frame_id.c_str(), calibration.right_frame_id.c_str(), error.what(),
                        calibration.baseline_m);
            return calibration.baseline_m;
        }
        if (tf_baseline > 0.0 && std::abs(tf_baseline - calibration.baseline_m) > 0.1 * tf_baseline)
        {
            RCLCPP_WARN(get_logger(),
                        "camera_info says the stereo baseline is %.4f m but TF puts %s %.4f m from %s; using "
                        "TF's. A wrong baseline scales every translation by the ratio (here %.2fx). Set "
                        "baseline_m to override.",
                        calibration.baseline_m, calibration.right_frame_id.c_str(), tf_baseline,
                        calibration.frame_id.c_str(), calibration.baseline_m / tf_baseline);
            return tf_baseline;
        }
        return calibration.baseline_m;
    }

    std::optional<std::string> OrbSlamOdometry::_write_settings(const stereo_calibration_t& calibration) const
    {
        const std::filesystem::path path =
            std::filesystem::temp_directory_path() /
            ("orb_slam_odometry_" + std::to_string(::getpid()) + "_" + std::string(get_name()) + ".yaml");
        std::ofstream out(path);
        // Fixed-point with decimals, always: ORB-SLAM3 rejects a real-valued key written as
        // a whole number ("Stereo.ThDepth: 40" reads as an int) and calls exit(), which
        // would take the camera container down with it. Integer keys are ints below.
        out << std::fixed << std::setprecision(6);
        // "Rectified": both images already on one image plane, so one set of intrinsics
        // and the baseline is the whole calibration. Viewer.* is read even headless.
        out << "%YAML:1.0\n"
            << "File.version: \"1.0\"\n"
            << "Camera.type: \"Rectified\"\n"
            << "Camera1.fx: " << calibration.fx << "\n"
            << "Camera1.fy: " << calibration.fy << "\n"
            << "Camera1.cx: " << calibration.cx << "\n"
            << "Camera1.cy: " << calibration.cy << "\n"
            << "Camera.width: " << calibration.width << "\n"
            << "Camera.height: " << calibration.height << "\n"
            << "Camera.fps: " << _camera_fps << "\n"
            << "Camera.RGB: 0\n"
            << "Stereo.b: " << calibration.baseline_m << "\n"
            << "Stereo.ThDepth: " << _th_depth << "\n"
            << "ORBextractor.nFeatures: " << _n_features << "\n"
            << "ORBextractor.scaleFactor: " << _scale_factor << "\n"
            << "ORBextractor.nLevels: " << _n_levels << "\n"
            << "ORBextractor.iniThFAST: " << _ini_th_fast << "\n"
            << "ORBextractor.minThFAST: " << _min_th_fast << "\n"
            << "Viewer.KeyFrameSize: 0.05\nViewer.KeyFrameLineWidth: 1.0\nViewer.GraphLineWidth: 0.9\n"
            << "Viewer.PointSize: 2.0\nViewer.CameraSize: 0.08\nViewer.CameraLineWidth: 3.0\n"
            << "Viewer.ViewpointX: 0.0\nViewer.ViewpointY: -0.7\nViewer.ViewpointZ: -1.8\n"
            << "Viewer.ViewpointF: 500.0\n"
            << "loopClosing: " << (_loop_closing ? 1 : 0) << "\n";
        out.close();
        if (!out)
        {
            RCLCPP_FATAL(get_logger(), "Could not write ORB-SLAM3 settings to %s", path.c_str());
            return std::nullopt;
        }
        return path.string();
    }

    void OrbSlamOdometry::_track(const stereo_frame_t& frame)
    {
        interfaces::msg::OrbSlamOdometryInfo info;
        info.header.stamp = frame.stamp;
        info.header.frame_id = _odometry_publisher->get_sensor_frame_id();

        const auto start = std::chrono::steady_clock::now();
        const Sophus::SE3f camera_from_world =
            _slam->TrackStereo(frame.left->image, frame.right->image, frame.stamp.seconds());
        info.runtime_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

        const int state = _slam->GetTrackingState();
        info.tracking_state = static_cast<int8_t>(state);
        int tracked = 0;
        for (const auto* point : _slam->GetTrackedMapPoints())
        {
            tracked += point != nullptr ? 1 : 0;
        }
        info.tracked_map_points = tracked;
        // Called every pass so a change is consumed when it happens, not later.
        info.map_changed = _slam->MapChanged();

        const bool tracking = state == ORB_SLAM3::Tracking::OK || state == ORB_SLAM3::Tracking::OK_KLT;
        if (!tracking || info.map_changed)
        {
            // Lost, initialising or relocalising, or the map moved: the next good frame
            // starts a new run of steps rather than stepping across the gap.
            _last_camera_pose.reset();
            if (!tracking)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), WARN_THROTTLE_MS,
                                     "ORB-SLAM3 not tracking (state %d)", state);
            }
            info.position_variance = LOST_TRACKING_VARIANCE;
            _publish_no_step(frame.stamp);
            _info_publisher->publish(info);
            return;
        }

        const Sophus::SE3f camera_pose = camera_from_world.inverse();
        if (!_last_camera_pose)
        {
            _last_camera_pose = camera_pose;
            info.position_variance = LOST_TRACKING_VARIANCE;
            _publish_no_step(frame.stamp);
            _info_publisher->publish(info);
            return;
        }

        // The camera's motion since the last tracked frame, in the previous camera frame:
        // the same quantity libviso2 produced, which OdometryPublisher integrates.
        const Sophus::SE3f step = _last_camera_pose->inverse() * camera_pose;
        _last_camera_pose = camera_pose;
        const double translation = step.translation().norm();
        const double rotation = step.so3().log().norm();
        if (translation > _max_step_translation_m || rotation > _max_step_rotation_rad)
        {
            // A jump ORB-SLAM3 did not flag: discard it rather than teleport the odometry.
            info.step_rejected = true;
            info.position_variance = LOST_TRACKING_VARIANCE;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), WARN_THROTTLE_MS,
                                 "Discarded a %.2f m / %.2f rad step as a discontinuity", translation, rotation);
            _publish_no_step(frame.stamp);
            _info_publisher->publish(info);
            return;
        }

        if (tracked < _min_map_points)
        {
            info.position_variance = LOST_TRACKING_VARIANCE;
            _publish_no_step(frame.stamp);
            _info_publisher->publish(info);
            return;
        }
        const double scale = std::clamp(static_cast<double>(_reference_map_points) / std::max(tracked, 1), 1.0,
                                        _max_covariance_scale);
        _odometry_publisher->set_pose_covariance(
            _diagonal_covariance(_position_variance, _orientation_variance, scale));
        _odometry_publisher->set_twist_covariance(
            _diagonal_covariance(_linear_velocity_variance, _angular_velocity_variance, scale));
        _odometry_publisher->integrate_and_publish(to_tf(step), frame.stamp);
        info.step_published = true;
        info.position_variance = _position_variance * scale;
        _info_publisher->publish(info);
    }

    void OrbSlamOdometry::_publish_no_step(const rclcpp::Time& stamp)
    {
        const auto lost = _diagonal_covariance(LOST_TRACKING_VARIANCE, LOST_TRACKING_VARIANCE, 1.0);
        _odometry_publisher->set_pose_covariance(lost);
        _odometry_publisher->set_twist_covariance(lost);
        _odometry_publisher->integrate_and_publish(tf2::Transform::getIdentity(), stamp);
    }

    std::array<double, 36> OrbSlamOdometry::_diagonal_covariance(double linear, double angular, double scale)
    {
        std::array<double, 36> covariance{};
        for (std::size_t i = 0; i < 3; ++i)
        {
            covariance[(i * 6) + i] = linear * scale;
            covariance[((i + 3) * 6) + (i + 3)] = angular * scale;
        }
        return covariance;
    }

}  // namespace vision

RCLCPP_COMPONENTS_REGISTER_NODE(vision::OrbSlamOdometry)
