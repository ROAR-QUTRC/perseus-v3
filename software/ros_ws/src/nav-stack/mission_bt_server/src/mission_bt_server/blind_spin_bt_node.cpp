/// @file blind_spin_bt_node.cpp
/// @brief Implementation and BT.CPP plugin registration for BlindSpinBtNode.

#include "mission_bt_server/mission_bt_server/blind_spin_bt_node.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <cmath>
#include <thread>

namespace mission_bt_server
{

    namespace
    {
        constexpr auto SPIN_BUDGET = std::chrono::milliseconds(5);
        // Repeat the command at 20 Hz, like escape_reverse.py: twist_mux times a topic out.
        constexpr auto SEND_PERIOD = std::chrono::milliseconds(50);
        constexpr auto ODOM_WAIT = std::chrono::seconds(2);
        constexpr int STOP_REPEATS = 5;

        double yaw_of(const geometry_msgs::msg::Quaternion& q)
        {
            return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        }
    }  // namespace

    BlindSpinBtNode::BlindSpinBtNode(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::StatefulActionNode(name, conf)
    {
        _node = config().blackboard->get<rclcpp::Node::SharedPtr>("@node");
        _callback_group =
            _node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        _executor.add_callback_group(_callback_group, _node->get_node_base_interface());
    }

    BT::PortsList BlindSpinBtNode::providedPorts()
    {
        return {
            BT::InputPort<double>("spin_dist", "radians to turn, positive left"),
            BT::InputPort<double>("speed_rad_s", 0.4,
                                  "turn rate; 0.4 clears the ESCs' stall, as nav2's Spin does"),
            BT::InputPort<double>("time_allowance_s", 40.0, "fail if the turn takes longer"),
            BT::InputPort<std::string>("topic", "cmd_vel_nav_stamped",
                                       "twist_mux's navigation input"),
        };
    }

    BT::NodeStatus BlindSpinBtNode::onStart()
    {
        double dist = 0.0, allowance = 40.0;
        if (!getInput("spin_dist", dist))
        {
            RCLCPP_ERROR(_node->get_logger(), "BlindSpin: no spin_dist");
            return BT::NodeStatus::FAILURE;
        }
        getInput("speed_rad_s", _speed);
        getInput("time_allowance_s", allowance);
        std::string topic = "cmd_vel_nav_stamped";
        getInput("topic", topic);
        _speed = std::abs(_speed);
        if (_speed <= 0.0 || dist == 0.0)
        {
            RCLCPP_ERROR(_node->get_logger(), "BlindSpin: needs a non-zero spin_dist and speed");
            return BT::NodeStatus::FAILURE;
        }
        _target = dist;

        if (!_publisher || topic != _topic)
        {
            _topic = topic;
            _publisher = _node->create_publisher<geometry_msgs::msg::TwistStamped>(_topic, 10);
        }
        _have_yaw = false;
        _turned = 0.0;
        rclcpp::SubscriptionOptions options;
        options.callback_group = _callback_group;
        // Best effort connects to either kind of publisher.
        _odom_subscription = _node->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
            {
                const double yaw = yaw_of(msg->pose.pose.orientation);
                if (_have_yaw)
                {
                    _turned += std::remainder(yaw - _last_yaw, 2.0 * M_PI);
                }
                _have_yaw = true;
                _last_yaw = yaw;
            },
            options);

        const auto now = std::chrono::steady_clock::now();
        _odom_deadline = now + ODOM_WAIT;
        _deadline = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(allowance));
        _last_send = now - SEND_PERIOD;
        RCLCPP_WARN(_node->get_logger(),
                    "BlindSpin: turning %.0f deg at %.2f rad/s on %s, no collision check",
                    dist * 180.0 / M_PI, _speed, _topic.c_str());
        return onRunning();
    }

    BT::NodeStatus BlindSpinBtNode::onRunning()
    {
        _executor.spin_some(SPIN_BUDGET);
        const auto now = std::chrono::steady_clock::now();

        // Without odometry there is no way to know when to stop, so never start.
        if (!_have_yaw)
        {
            if (now >= _odom_deadline)
            {
                RCLCPP_ERROR(_node->get_logger(), "BlindSpin: no /odometry/filtered, not turning");
                _stop();
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;
        }
        if (std::abs(_turned) >= std::abs(_target))
        {
            RCLCPP_INFO(_node->get_logger(), "BlindSpin: turned %.0f deg",
                        _turned * 180.0 / M_PI);
            _stop();
            return BT::NodeStatus::SUCCESS;
        }
        if (now >= _deadline)
        {
            RCLCPP_ERROR(_node->get_logger(), "BlindSpin: only %.0f deg turned in time",
                         _turned * 180.0 / M_PI);
            _stop();
            return BT::NodeStatus::FAILURE;
        }
        if (now - _last_send >= SEND_PERIOD)
        {
            _send(_target > 0.0 ? _speed : -_speed);
            _last_send = now;
        }
        return BT::NodeStatus::RUNNING;
    }

    void BlindSpinBtNode::onHalted()
    {
        _stop();
    }

    void BlindSpinBtNode::_send(double angular_z)
    {
        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = _node->now();
        msg.header.frame_id = "base_link";
        msg.twist.angular.z = angular_z;
        _publisher->publish(msg);
    }

    void BlindSpinBtNode::_stop()
    {
        _odom_subscription.reset();
        if (!_publisher)
        {
            return;
        }
        // Zeros, a few times, so the diff drive stops rather than coasting on the last
        // turn command. This blocks the tree for about a quarter of a second.
        for (int i = 0; i < STOP_REPEATS; ++i)
        {
            _send(0.0);
            std::this_thread::sleep_for(SEND_PERIOD);
        }
    }

}  // namespace mission_bt_server

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<mission_bt_server::BlindSpinBtNode>("BlindSpin");
}
