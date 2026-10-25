#pragma once

/// @file wall_side_bt_node.hpp
/// @brief BT.CPP leaf that picks which way to spin from the nearest wall.

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>

namespace mission_bt_server
{

    /// @brief Looks at the local terrain costmap and says which way to turn: toward
    /// the side the nearest wall is on. Writes spin_dist, step_deg in radians, positive
    /// (counter-clockwise, left) or negative (right) for nav2's Spin node to take.
    ///
    /// The rover starts a mission at a random heading inside the starting zone, and the
    /// rail with the fiducials is on the wall. Turning toward the closer wall sweeps
    /// the camera along it, rather than across the open arena.
    ///
    /// "Wall" is any lethal cell of /local_costmap_terrain within range_m, which is
    /// what the traversability node makes of anything standing proud of the ground, and
    /// the node clears the rover's own footprint from it. Each cell is put on the
    /// left or the right by its bearing from the rover's heading (odometry's pose, in
    /// the same odom frame as the grid), and the side with the nearer cell wins. A wall
    /// dead ahead or behind lands on whichever side its nearest cell is. With nothing
    /// in range, or no costmap or odometry, it turns left and says so.
    ///
    /// A StatefulActionNode, because the costmap and odometry that matter are the ones
    /// from after the tick: the node is built with the tree, a moment before it runs.
    /// Runs its own executor on its own callback group, like MoveBucket.
    class WallSideBtNode : public BT::StatefulActionNode
    {
    public:
        WallSideBtNode(const std::string& name, const BT::NodeConfiguration& conf);

        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        /// @brief Writes the outputs for @p left (true) or right and returns SUCCESS.
        BT::NodeStatus _finish(bool left, const char* why);

        /// @brief Which side the nearest lethal cell is on, or nullopt with none in range.
        std::optional<bool> _nearest_side() const;

        rclcpp::Node::SharedPtr _node;
        rclcpp::CallbackGroup::SharedPtr _callback_group;
        rclcpp::executors::SingleThreadedExecutor _executor;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _grid_subscription;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_subscription;

        // Written by the callbacks inside _executor.spin_some(), on the tree's thread.
        nav_msgs::msg::OccupancyGrid::ConstSharedPtr _grid;
        nav_msgs::msg::Odometry::ConstSharedPtr _odom;

        double _step_rad{0.0};
        double _range_m{4.0};
        int _min_cost{99};
        std::chrono::steady_clock::time_point _deadline;
    };

}  // namespace mission_bt_server
