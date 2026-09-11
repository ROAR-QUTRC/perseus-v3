/// @file detection_overlay.cpp
/// @brief Implementation of the real-time detection overlay node.

#include "vision/detection_overlay/detection_overlay.hpp"

#include <functional>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>

#include "vision/common/detection_renderer.hpp"

namespace vision
{
    namespace
    {
        /// @brief Queue depth used for the image topics.
        ///
        /// Deliberately shallow: if the node cannot keep up, dropping older frames
        /// keeps the overlay current rather than publishing a growing backlog of stale
        /// images.
        constexpr int IMAGE_QOS_DEPTH = 1;
        /// @brief Queue depth used for the detection topics.
        constexpr int DETECTION_QOS_DEPTH = 5;
        /// @brief Queue depth used for the plan and camera_info topics.
        constexpr int PATH_QOS_DEPTH = 5;
        /// @brief JPEG quality used when re-encoding the annotated compressed image.
        constexpr int JPEG_QUALITY = 90;
        /// @brief Colour the plan is drawn in, in OpenCV BGR order (blue).
        const cv::Scalar PATH_COLOR(255, 0, 0);
        /// @brief Line thickness used to draw the plan, in pixels.
        constexpr int PATH_LINE_THICKNESS = 3;
        /// @brief Minimum in-front-of-camera depth a plan point must have to be
        /// projected, in metres. Guards the pinhole divide against blowing up near
        /// the camera plane.
        constexpr double MIN_PATH_POINT_DEPTH_M = 0.05;
        /// @brief Minimum gap between repeated warning logs, in milliseconds.
        constexpr int64_t LOG_THROTTLE_MS = 2000;
        /// @brief Left margin used for the staleness readout, in pixels.
        constexpr int STALENESS_MARGIN_PX = 10;
        /// @brief Baseline of the staleness readout, in pixels from the top.
        constexpr int STALENESS_Y_PX = 20;
        /// @brief Font scale used for the staleness readout.
        constexpr double STALENESS_FONT_SCALE = 0.5;
        /// @brief Stroke thickness used for the staleness readout.
        constexpr int STALENESS_THICKNESS = 1;
        /// @brief Decimal places shown for the detection age.
        constexpr int STALENESS_PRECISION = 2;

        /// @brief Encoding the overlay works in and republishes.
        const std::string IMAGE_ENCODING = "bgr8";
    }  // namespace

    DetectionOverlay::DetectionOverlay(const rclcpp::NodeOptions& options)
        : Node("detection_overlay", options)
    {
        _input_image_topic = declare_parameter<std::string>(
            "input_image_topic", DEFAULT_INPUT_IMAGE_TOPIC);
        _output_image_topic = declare_parameter<std::string>(
            "output_image_topic", DEFAULT_OUTPUT_IMAGE_TOPIC);
        _detection_topics = declare_parameter<std::vector<std::string>>(
            "detection_topics", DEFAULT_DETECTION_TOPICS);
        _max_detection_age_s = declare_parameter<double>("max_detection_age_s",
                                                         DEFAULT_MAX_DETECTION_AGE_S);
        _input_compressed =
            declare_parameter<bool>("input_compressed", DEFAULT_INPUT_COMPRESSED);
        _output_compressed =
            declare_parameter<bool>("output_compressed", DEFAULT_OUTPUT_COMPRESSED);
        _should_show_staleness =
            declare_parameter<bool>("show_staleness", DEFAULT_SHOULD_SHOW_STALENESS);
        _path_topic = declare_parameter<std::string>("path_topic", DEFAULT_PATH_TOPIC);
        _camera_info_topic = declare_parameter<std::string>(
            "camera_info_topic", DEFAULT_CAMERA_INFO_TOPIC);
        _should_show_path =
            declare_parameter<bool>("show_path", DEFAULT_SHOULD_SHOW_PATH);

        _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer);

        if (_input_compressed)
        {
            _compressed_image_subscription =
                create_subscription<sensor_msgs::msg::CompressedImage>(
                    _input_image_topic + "/compressed", IMAGE_QOS_DEPTH,
                    std::bind(&DetectionOverlay::_compressed_image_callback, this,
                              std::placeholders::_1));
        }
        else
        {
            _image_subscription = create_subscription<sensor_msgs::msg::Image>(
                _input_image_topic, IMAGE_QOS_DEPTH,
                std::bind(&DetectionOverlay::_image_callback, this,
                          std::placeholders::_1));
        }

        if (_output_compressed)
        {
            _compressed_image_publisher =
                create_publisher<sensor_msgs::msg::CompressedImage>(
                    _output_image_topic + "/compressed", IMAGE_QOS_DEPTH);
        }
        else
        {
            _image_publisher = create_publisher<sensor_msgs::msg::Image>(
                _output_image_topic, IMAGE_QOS_DEPTH);
        }

        _detection_subscriptions.reserve(_detection_topics.size());
        for (const auto& topic : _detection_topics)
        {
            _detection_subscriptions.push_back(
                create_subscription<interfaces::msg::DetectionArray>(
                    topic, DETECTION_QOS_DEPTH,
                    [this,
                     topic](const interfaces::msg::DetectionArray::SharedPtr msg)
                    {
                        _detections_callback(topic, msg);
                    }));

            RCLCPP_INFO(get_logger(), "Overlaying detections from: %s", topic.c_str());
        }

        _path_subscription = create_subscription<nav_msgs::msg::Path>(
            _path_topic, PATH_QOS_DEPTH,
            std::bind(&DetectionOverlay::_path_callback, this, std::placeholders::_1));
        _camera_info_subscription = create_subscription<sensor_msgs::msg::CameraInfo>(
            _camera_info_topic, PATH_QOS_DEPTH,
            std::bind(&DetectionOverlay::_camera_info_callback, this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "DetectionOverlay ready — %s -> %s",
                    _input_image_topic.c_str(), _output_image_topic.c_str());
    }

    void DetectionOverlay::_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(_path_mutex);
        _latest_path = *msg;
    }

    void DetectionOverlay::_camera_info_callback(
        const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(_camera_info_mutex);
        _latest_camera_info = *msg;
        _has_camera_info = true;
    }

    void DetectionOverlay::_draw_path(cv::Mat& frame)
    {
        nav_msgs::msg::Path path;
        {
            std::lock_guard<std::mutex> lock(_path_mutex);
            path = _latest_path;
        }
        if (path.poses.empty())
        {
            return;
        }

        sensor_msgs::msg::CameraInfo camera_info;
        {
            std::lock_guard<std::mutex> lock(_camera_info_mutex);
            if (!_has_camera_info)
            {
                return;
            }
            camera_info = _latest_camera_info;
        }

        geometry_msgs::msg::TransformStamped path_to_camera;
        try
        {
            path_to_camera = _tf_buffer->lookupTransform(
                camera_info.header.frame_id, path.header.frame_id, tf2::TimePointZero);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), LOG_THROTTLE_MS,
                                 "Waiting for %s -> %s: %s", path.header.frame_id.c_str(),
                                 camera_info.header.frame_id.c_str(), ex.what());
            return;
        }

        const double fx = camera_info.k[0];
        const double fy = camera_info.k[4];
        const double cx = camera_info.k[2];
        const double cy = camera_info.k[5];

        bool has_previous_point = false;
        cv::Point previous_point;

        for (const auto& pose_stamped : path.poses)
        {
            geometry_msgs::msg::PointStamped point_in;
            point_in.header = path.header;
            point_in.point = pose_stamped.pose.position;

            geometry_msgs::msg::PointStamped point_in_camera;
            tf2::doTransform(point_in, point_in_camera, path_to_camera);

            const double z = point_in_camera.point.z;
            if (z < MIN_PATH_POINT_DEPTH_M)
            {
                // Behind (or right on top of) the camera: break the line here rather
                // than connecting across the discontinuity.
                has_previous_point = false;
                continue;
            }

            const cv::Point pixel(
                static_cast<int>(std::lround(fx * point_in_camera.point.x / z + cx)),
                static_cast<int>(std::lround(fy * point_in_camera.point.y / z + cy)));

            if (has_previous_point)
            {
                cv::line(frame, previous_point, pixel, PATH_COLOR, PATH_LINE_THICKNESS);
            }
            previous_point = pixel;
            has_previous_point = true;
        }
    }

    void DetectionOverlay::_detections_callback(
        const std::string& topic,
        const interfaces::msg::DetectionArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(_detections_mutex);
        _latest_detections[topic] = *msg;
    }

    std::size_t DetectionOverlay::_draw_cached_detections(cv::Mat& frame,
                                                          const rclcpp::Time& now)
    {
        // Copy under the lock, then draw outside it, so a slow render never stalls a
        // detector's callback.
        std::vector<interfaces::msg::DetectionArray> fresh_detections;
        {
            std::lock_guard<std::mutex> lock(_detections_mutex);
            fresh_detections.reserve(_latest_detections.size());
            for (const auto& [topic, detections] : _latest_detections)
            {
                const rclcpp::Time stamp(detections.header.stamp, now.get_clock_type());
                const double age_s = (now - stamp).seconds();
                if (age_s < 0.0 || age_s > _max_detection_age_s)
                {
                    continue;
                }
                fresh_detections.push_back(detections);
            }
        }

        std::size_t drawn_count = 0;
        for (const auto& detections : fresh_detections)
        {
            draw_detections(frame, detections);
            drawn_count += detections.detections.size();
        }

        if (_should_show_staleness)
        {
            double oldest_age_s = 0.0;
            for (const auto& detections : fresh_detections)
            {
                const rclcpp::Time stamp(detections.header.stamp, now.get_clock_type());
                oldest_age_s = std::max(oldest_age_s, (now - stamp).seconds());
            }

            std::ostringstream staleness_stream;
            staleness_stream << "detections: " << drawn_count << "  age: " << std::fixed
                             << std::setprecision(STALENESS_PRECISION) << oldest_age_s
                             << "s";
            cv::putText(frame, staleness_stream.str(),
                        cv::Point(STALENESS_MARGIN_PX, STALENESS_Y_PX),
                        cv::FONT_HERSHEY_SIMPLEX, STALENESS_FONT_SCALE,
                        cv::Scalar(255, 255, 255), STALENESS_THICKNESS);
        }

        return drawn_count;
    }

    void DetectionOverlay::_image_callback(
        const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat frame;
        try
        {
            frame = cv_bridge::toCvCopy(msg, IMAGE_ENCODING)->image;
        }
        catch (const cv_bridge::Exception& e)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), LOG_THROTTLE_MS,
                                  "cv_bridge exception: %s", e.what());
            return;
        }

        if (_should_show_path)
        {
            _draw_path(frame);
        }
        _draw_cached_detections(frame, msg->header.stamp);
        _publish_frame(frame, msg->header);
    }

    void DetectionOverlay::_compressed_image_callback(
        const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        cv::Mat frame;
        try
        {
            frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            if (frame.empty())
            {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), LOG_THROTTLE_MS,
                                      "Failed to decode compressed image");
                return;
            }
        }
        catch (const cv::Exception& e)
        {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), LOG_THROTTLE_MS,
                                  "OpenCV exception: %s", e.what());
            return;
        }

        if (_should_show_path)
        {
            _draw_path(frame);
        }
        _draw_cached_detections(frame, msg->header.stamp);
        _publish_frame(frame, msg->header);
    }

    void DetectionOverlay::_publish_frame(const cv::Mat& frame,
                                          const std_msgs::msg::Header& header)
    {
        if (_output_compressed)
        {
            sensor_msgs::msg::CompressedImage compressed_msg;
            compressed_msg.header = header;
            compressed_msg.format = "jpeg";

            std::vector<uchar> buffer;
            const std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY,
                                                    JPEG_QUALITY};
            cv::imencode(".jpg", frame, buffer, encode_params);
            compressed_msg.data = std::move(buffer);

            _compressed_image_publisher->publish(compressed_msg);
        }
        else
        {
            _image_publisher->publish(
                *cv_bridge::CvImage(header, IMAGE_ENCODING, frame).toImageMsg());
        }
    }

}  // namespace vision

RCLCPP_COMPONENTS_REGISTER_NODE(vision::DetectionOverlay)
