/// @file height_crop.cpp
/// @brief Implementation of the height crop node.

#include "sensors/height_crop/height_crop.hpp"

#include <tf2/exceptions.h>

#include <chrono>
#include <cstring>
#include <functional>
#include <optional>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace sensors
{
    namespace
    {
        /// @brief Queue depth for the input and output topics.
        constexpr int QOS_DEPTH = 5;

        /// @brief Byte offset of a float32 field, if the cloud has one by that name.
        std::optional<std::size_t> float_offset(const sensor_msgs::msg::PointCloud2& msg,
                                                const std::string& name)
        {
            for (const auto& field : msg.fields)
            {
                if (field.name == name &&
                    field.datatype == sensor_msgs::msg::PointField::FLOAT32)
                {
                    return static_cast<std::size_t>(field.offset);
                }
            }
            return std::nullopt;
        }
    }  // namespace

    HeightCrop::HeightCrop(const rclcpp::NodeOptions& options)
        : rclcpp::Node("height_crop", options)
    {
        const std::string input_topic =
            this->declare_parameter("input_topic", DEFAULT_INPUT_TOPIC);
        const std::string output_topic =
            this->declare_parameter("output_topic", DEFAULT_OUTPUT_TOPIC);
        _reference_frame = this->declare_parameter("reference_frame", DEFAULT_REFERENCE_FRAME);
        _max_height_m = this->declare_parameter("max_height_m", DEFAULT_MAX_HEIGHT_M);
        _tf_timeout_s = this->declare_parameter("tf_timeout_s", DEFAULT_TF_TIMEOUT_S);
        // /Laser_map is latched (transient_local) and only published every few seconds;
        // latching the cropped copy too lets a late subscriber -- the base station
        // joining mid-run -- get the current map at once instead of waiting.
        const bool latched = this->declare_parameter("output_transient_local", false);

        _tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        _tf_listener = std::make_unique<tf2_ros::TransformListener>(*_tf_buffer);

        // Reliable and volatile: matches both the Livox driver's and BIEVR-LIO's
        // publishers, and best-effort subscribers downstream still connect to it.
        _subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, rclcpp::QoS(QOS_DEPTH),
            std::bind(&HeightCrop::_point_cloud_callback, this, std::placeholders::_1));

        rclcpp::QoS output_qos(latched ? 1 : QOS_DEPTH);
        if (latched)
        {
            output_qos.transient_local();
        }
        _publisher =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, output_qos);

        RCLCPP_INFO(this->get_logger(), "Height crop: %s -> %s, keeping z <= %.2f m in %s",
                    input_topic.c_str(), output_topic.c_str(), _max_height_m,
                    _reference_frame.c_str());
    }

    void HeightCrop::_point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        const auto callback_start = std::chrono::steady_clock::now();

        const auto x_offset = float_offset(*msg, "x");
        const auto y_offset = float_offset(*msg, "y");
        const auto z_offset = float_offset(*msg, "z");
        if (!x_offset || !y_offset || !z_offset)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "Point cloud has no float32 x/y/z fields; dropping it.");
            return;
        }

        // Height in the reference frame is one row of the transform:
        //   z_ref = r20 * x + r21 * y + r22 * z + tz
        // so only that row is kept. Identity when the cloud is already there.
        double r20 = 0.0;
        double r21 = 0.0;
        double r22 = 1.0;
        double tz = 0.0;
        if (msg->header.frame_id != _reference_frame)
        {
            geometry_msgs::msg::TransformStamped transform;
            try
            {
                transform = _tf_buffer->lookupTransform(
                    _reference_frame, msg->header.frame_id, msg->header.stamp,
                    rclcpp::Duration::from_seconds(_tf_timeout_s));
            }
            catch (const tf2::TransformException& error)
            {
                // Dropped, not passed through: an uncropped scan would put the ceiling
                // straight back into the costmaps this exists to keep it out of.
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "No %s -> %s transform at the scan stamp, dropping it: %s",
                                     msg->header.frame_id.c_str(), _reference_frame.c_str(),
                                     error.what());
                return;
            }
            const auto& q = transform.transform.rotation;
            r20 = 2.0 * (q.x * q.z - q.w * q.y);
            r21 = 2.0 * (q.y * q.z + q.w * q.x);
            r22 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
            tz = transform.transform.translation.z;
        }

        sensor_msgs::msg::PointCloud2 output;
        output.header = msg->header;
        output.height = 1;
        output.fields = msg->fields;
        output.is_bigendian = msg->is_bigendian;
        output.point_step = msg->point_step;
        output.is_dense = msg->is_dense;
        output.data.reserve(msg->data.size());

        const std::size_t point_count = static_cast<std::size_t>(msg->width) * msg->height;
        std::size_t kept = 0;
        for (std::size_t i = 0; i < point_count; ++i)
        {
            const std::size_t base = i * msg->point_step;
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            std::memcpy(&x, &msg->data[base + *x_offset], sizeof(float));
            std::memcpy(&y, &msg->data[base + *y_offset], sizeof(float));
            std::memcpy(&z, &msg->data[base + *z_offset], sizeof(float));
            // A NaN height fails this test and is dropped with the ceiling.
            if (!(r20 * x + r21 * y + r22 * z + tz <= _max_height_m))
            {
                continue;
            }
            output.data.insert(output.data.end(), msg->data.begin() + base,
                               msg->data.begin() + base + msg->point_step);
            ++kept;
        }
        output.width = static_cast<uint32_t>(kept);
        output.row_step = output.width * output.point_step;

        _publisher->publish(output);

        const auto elapsed_ms = std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - callback_start)
                                    .count();
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "Kept %zu/%zu points in %.1f ms", kept, point_count, elapsed_ms);
    }

}  // namespace sensors

RCLCPP_COMPONENTS_REGISTER_NODE(sensors::HeightCrop)
