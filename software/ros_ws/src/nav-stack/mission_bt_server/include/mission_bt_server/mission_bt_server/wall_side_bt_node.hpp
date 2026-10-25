#pragma once

/// @file wall_side_bt_node.hpp
/// @brief BT.CPP leaf that picks which way to spin from the nearest wall, and refuses
/// to if a wall is too close to turn in place.

#include <behaviortree_cpp/action_node.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace mission_bt_server
{

    /// @brief Looks at the lidar and says which way to turn: toward the side the nearest
    /// wall is on. Writes spin_dist, spin_deg in radians, positive (counter-clockwise,
    /// left) or negative (right), for BlindSpin to take. Fails, without writing it, if
    /// the nearest wall is closer than min_clearance_m to the rover's centre.
    ///
    /// The rover starts a mission at a random heading inside the starting zone, and the
    /// rail with the fiducials is on the wall. Turning toward the closer wall sweeps
    /// the camera along it, rather than across the open arena.
    ///
    /// Reads the raw cloud and not the terrain costmaps. local_traversability drops every
    /// return within min_range_m (1.7 m) of the lidar, because those are the rover, so no
    /// grid holds a wall nearer than that: exactly the walls that decide whether the
    /// rover can turn. Each point is moved into base_footprint, and counted as wall when
    /// it is between z_min_m and z_max_m (above the ground, below the ceiling) and outside
    /// the rover's own box (the footprint plus self_pad_m). The nearest wall point on the
    /// left and on the right, in plan from base_footprint, then decide the side, and the
    /// nearer of the two is the clearance. With no wall within range_m it turns left.
    ///
    /// A StatefulActionNode: it gathers frames for window_s from after the tick, since the
    /// node is built with the tree, a moment before it runs. TF is listened to on a
    /// thread of its own while it runs only. Runs its own executor on its own callback
    /// group, like MoveBucket.
    class WallSideBtNode : public BT::StatefulActionNode
    {
    public:
        WallSideBtNode(const std::string& name, const BT::NodeConfiguration& conf);

        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        void _on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

        /// @brief Stops listening and returns SUCCESS with the outputs for @p left.
        BT::NodeStatus _finish(bool left, const char* why);

        /// @brief Stops listening.
        void _release();

        rclcpp::Node::SharedPtr _node;
        rclcpp::CallbackGroup::SharedPtr _callback_group;
        rclcpp::executors::SingleThreadedExecutor _executor;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _subscription;
        std::shared_ptr<tf2_ros::Buffer> _tf_buffer;
        std::shared_ptr<tf2_ros::TransformListener> _tf_listener;

        // From the ports, per tick of onStart.
        double _spin_rad{0.0};
        double _range_m{4.0};
        double _min_clearance_m{0.0};
        double _z_min_m{0.2};
        double _z_max_m{1.2};
        double _box_x_min{0.0}, _box_x_max{0.0}, _box_half_width{0.0};
        double _window_s{0.6};
        std::string _topic;
        std::string _base_frame;

        // Running minima over the window, plan distance from base_footprint.
        double _nearest_left{0.0};
        double _nearest_right{0.0};
        int _frames{0};
        std::chrono::steady_clock::time_point _window_end;
        std::chrono::steady_clock::time_point _deadline;
    };

}  // namespace mission_bt_server
