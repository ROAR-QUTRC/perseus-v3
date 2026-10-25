/// @file wall_side_bt_node.cpp
/// @brief Implementation and BT.CPP plugin registration for WallSideBtNode.

#include "mission_bt_server/mission_bt_server/wall_side_bt_node.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mission_bt_server
{

    namespace
    {
        // How long to wait for the first usable cloud before giving up and defaulting.
        constexpr auto DATA_WAIT = std::chrono::seconds(4);
        constexpr auto SPIN_BUDGET = std::chrono::milliseconds(20);
        constexpr double INF = std::numeric_limits<double>::infinity();
    }  // namespace

    WallSideBtNode::WallSideBtNode(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::StatefulActionNode(name, conf)
    {
        _node = config().blackboard->get<rclcpp::Node::SharedPtr>("@node");
        _callback_group =
            _node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        _executor.add_callback_group(_callback_group, _node->get_node_base_interface());
    }

    BT::PortsList WallSideBtNode::providedPorts()
    {
        return {
            BT::InputPort<double>("spin_deg", 360.0, "size of the turn written to spin_dist"),
            BT::InputPort<double>("range_m", 4.0, "ignore walls farther than this"),
            BT::InputPort<double>("min_clearance_m", 0.0,
                                  "fail when a wall is nearer than this to the rover's "
                                  "center; 0 or less never fails"),
            BT::InputPort<std::string>("topic", "/livox/lidar/cropped",
                                       "the lidar's PointCloud2"),
            BT::InputPort<std::string>("base_frame", "base_footprint",
                                       "the frame the rover turns about"),
            BT::InputPort<double>("z_min_m", 0.2, "lowest a wall point may be above the ground"),
            BT::InputPort<double>("z_max_m", 1.2, "highest a wall point may be"),
            BT::InputPort<double>("box_x_min_m", -0.775,
                                  "the rover's own box, behind base_footprint, before padding"),
            BT::InputPort<double>("box_x_max_m", 0.9, "ahead of base_footprint"),
            BT::InputPort<double>("box_half_width_m", 0.435, "either side of base_footprint"),
            BT::InputPort<double>("self_pad_m", 0.15, "padding added to that box on every side"),
            BT::InputPort<double>("window_s", 0.6, "how long to gather clouds for"),
            BT::OutputPort<double>("spin_dist",
                                   "spin_deg in radians: positive left, negative right"),
            BT::OutputPort<std::string>("side", "left or right"),
        };
    }

    BT::NodeStatus WallSideBtNode::onStart()
    {
        double spin_deg = 360.0, pad = 0.15;
        getInput("spin_deg", spin_deg);
        getInput("range_m", _range_m);
        getInput("min_clearance_m", _min_clearance_m);
        getInput("topic", _topic);
        getInput("base_frame", _base_frame);
        getInput("z_min_m", _z_min_m);
        getInput("z_max_m", _z_max_m);
        getInput("box_x_min_m", _box_x_min);
        getInput("box_x_max_m", _box_x_max);
        getInput("box_half_width_m", _box_half_width);
        getInput("self_pad_m", pad);
        getInput("window_s", _window_s);
        _spin_rad = std::abs(spin_deg) * M_PI / 180.0;
        _box_x_min -= pad;
        _box_x_max += pad;
        _box_half_width += pad;

        _nearest_left = _nearest_right = INF;
        _frames = 0;
        const auto now = std::chrono::steady_clock::now();
        _deadline = now + DATA_WAIT;
        _window_end = std::chrono::steady_clock::time_point::max();

        _tf_buffer = std::make_shared<tf2_ros::Buffer>(_node->get_clock());
        _tf_listener = std::make_shared<tf2_ros::TransformListener>(*_tf_buffer, _node, true);
        rclcpp::SubscriptionOptions options;
        options.callback_group = _callback_group;
        _subscription = _node->create_subscription<sensor_msgs::msg::PointCloud2>(
            _topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
            { _on_cloud(std::move(msg)); },
            options);
        return onRunning();
    }

    void WallSideBtNode::_on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
    {
        tf2::Transform to_base;
        try
        {
            const auto transform = _tf_buffer->lookupTransform(
                _base_frame, msg->header.frame_id, tf2::TimePointZero);
            tf2::fromMsg(transform.transform, to_base);
        }
        catch (const tf2::TransformException&)
        {
            return;  // TF is still filling; the next cloud will do.
        }

        sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
        for (; x != x.end(); ++x, ++y, ++z)
        {
            if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z))
            {
                continue;
            }
            const tf2::Vector3 p = to_base * tf2::Vector3(*x, *y, *z);
            if (p.z() < _z_min_m || p.z() > _z_max_m)
            {
                continue;
            }
            if (p.x() > _box_x_min && p.x() < _box_x_max && std::abs(p.y()) < _box_half_width)
            {
                continue;  // the rover itself
            }
            const double distance = std::hypot(p.x(), p.y());
            if (distance > _range_m)
            {
                continue;
            }
            double& nearest = p.y() > 0.0 ? _nearest_left : _nearest_right;
            nearest = std::min(nearest, distance);
        }
        if (++_frames == 1)
        {
            _window_end = std::chrono::steady_clock::now() +
                          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(_window_s));
        }
    }

    BT::NodeStatus WallSideBtNode::onRunning()
    {
        _executor.spin_some(SPIN_BUDGET);
        const auto now = std::chrono::steady_clock::now();
        if (_frames > 0 && now >= _window_end)
        {
            RCLCPP_INFO(_node->get_logger(),
                        "WallSide: nearest wall %.2f m left, %.2f m right of %s (inf: none "
                        "within %.1f m)",
                        _nearest_left, _nearest_right, _base_frame.c_str(), _range_m);
            const double nearest = std::min(_nearest_left, _nearest_right);
            if (_min_clearance_m > 0.0 && nearest < _min_clearance_m)
            {
                RCLCPP_ERROR(_node->get_logger(),
                             "WallSide: a wall is %.2f m from the rover's center, and turning "
                             "in place needs %.2f m. Move the rover away from it and start "
                             "again.",
                             nearest, _min_clearance_m);
                _release();
                return BT::NodeStatus::FAILURE;
            }
            if (nearest == INF)
            {
                return _finish(true, "no wall in range, defaulting");
            }
            return _finish(_nearest_left <= _nearest_right, "nearest wall");
        }
        if (_frames == 0 && now >= _deadline)
        {
            // Without a lidar there is no wall to check, and no clearance to refuse on.
            RCLCPP_WARN(_node->get_logger(),
                        "WallSide: no usable cloud on %s (or no TF to %s) in %ld s; turning "
                        "left WITHOUT a clearance check",
                        _topic.c_str(), _base_frame.c_str(),
                        static_cast<long>(std::chrono::duration_cast<std::chrono::seconds>(
                                              DATA_WAIT)
                                              .count()));
            return _finish(true, "no lidar, defaulting");
        }
        return BT::NodeStatus::RUNNING;
    }

    void WallSideBtNode::onHalted()
    {
        _release();
    }

    void WallSideBtNode::_release()
    {
        _subscription.reset();
        _tf_listener.reset();
        _tf_buffer.reset();
    }

    BT::NodeStatus WallSideBtNode::_finish(bool left, const char* why)
    {
        setOutput("spin_dist", left ? _spin_rad : -_spin_rad);
        setOutput("side", std::string(left ? "left" : "right"));
        RCLCPP_INFO(_node->get_logger(), "WallSide: spinning %s (%s)", left ? "left" : "right",
                    why);
        _release();
        return BT::NodeStatus::SUCCESS;
    }

}  // namespace mission_bt_server

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<mission_bt_server::WallSideBtNode>("WallSide");
}
