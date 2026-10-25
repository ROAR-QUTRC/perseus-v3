#pragma once

/// @file blind_spin_bt_node.hpp
/// @brief BT.CPP leaf that turns the rover on the spot without a costmap check.

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mission_bt_server
{

    /// @brief Turns spin_dist radians (positive left) at speed_rad_s by publishing
    /// angular velocity on twist_mux's navigation input, and measures the turn from the
    /// filtered odometry's yaw. SUCCESS when the turn is done, FAILURE if it takes longer
    /// than time_allowance_s or there is no odometry. Halting it, as a Parallel does when
    /// WaitForMarkers wins, stops the rover.
    ///
    /// Why not nav2's Spin. Spin collision-checks the footprint at every pose it turns
    /// into, with the costmap's lethal band around each wall, and the rover's circle
    /// (0.93 m from base_footprint) plus that band does not fit in the 2.25 m starting
    /// zone, so it refuses a full turn there. This node does not check, which is safe
    /// only if the walls are far enough, and that is WallSide's min_clearance_m. It is
    /// the same trade as autonomy_bringup's escape_reverse.py.
    ///
    /// Publishes on cmd_vel_nav_stamped (priority 10), so the joystick, keyboard and web
    /// UI all still take over at once, and stops with zeros, since the diff drive
    /// coasts on its last command otherwise. Runs its own executor on its own callback
    /// group, like MoveBucket.
    class BlindSpinBtNode : public BT::StatefulActionNode
    {
    public:
        BlindSpinBtNode(const std::string& name, const BT::NodeConfiguration& conf);

        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        void _send(double angular_z);
        /// @brief Publishes zeros a few times, and stops listening to odometry.
        void _stop();

        rclcpp::Node::SharedPtr _node;
        rclcpp::CallbackGroup::SharedPtr _callback_group;
        rclcpp::executors::SingleThreadedExecutor _executor;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _publisher;
        std::string _topic;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_subscription;

        // Written by the odometry callback inside _executor.spin_some().
        bool _have_yaw{false};
        double _last_yaw{0.0};
        double _turned{0.0};

        double _target{0.0};
        double _speed{0.4};
        std::chrono::steady_clock::time_point _deadline;
        std::chrono::steady_clock::time_point _odom_deadline;
        std::chrono::steady_clock::time_point _last_send;
    };

}  // namespace mission_bt_server
