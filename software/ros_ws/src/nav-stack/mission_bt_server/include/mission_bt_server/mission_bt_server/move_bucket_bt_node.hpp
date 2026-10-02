#pragma once

/// @file move_bucket_bt_node.hpp
/// @brief BT.CPP leaf that moves the excavation bucket to a lift/tilt/jaw pose.

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>

namespace mission_bt_server
{

    /// @brief Sends one FollowJointTrajectory goal to the bucket's
    /// joint_trajectory_controller (payloads' bucket.launch.py, or the simulation's)
    /// and succeeds when the controller reports the bucket at the pose.
    ///
    /// The pose is in degrees, in the bucket joint conventions: lift 0 with the arms
    /// level and positive lowering, tilt 0 level and positive dumping, jaw 0 closed
    /// and positive opening. Each of lift_deg, tilt_deg and jaw_deg is optional: only
    /// the joints given are commanded, and the controller holds the others where they
    /// are, so <MoveBucket jaw_deg="36"/> opens the jaw and nothing else. The goal is one point reached duration_s after it is
    /// sent; the controller's own goal tolerances (bucket_controller.yaml) decide
    /// whether it arrived.
    ///
    /// Deliberately a plain StatefulActionNode rather than nav2's BtActionNode:
    /// BtActionNode waits for its action server when the tree is built and throws if
    /// it is absent, which would stop every mission from even starting on a rover
    /// with no bucket controller running - including missions that never move the
    /// bucket. This node connects on its first tick instead, so only a tree that
    /// actually ticks it needs the bucket, and then fails with a log line saying so.
    ///
    /// Spins its own executor on its own callback group, like nav2's BT nodes, so the
    /// action client is serviced on the tree's thread and nothing else needs spinning.
    /// Halting it (the mission being stopped) cancels the goal, which leaves the
    /// controller holding the bucket where it is.
    class MoveBucketBtNode : public BT::StatefulActionNode
    {
    public:
        using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
        using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

        MoveBucketBtNode(const std::string& name, const BT::NodeConfiguration& conf);

        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        /// @brief Creates the action client for @p action_name, replacing one made
        /// for a different name.
        void _make_client(const std::string& action_name);

        /// @brief Services the action client's callbacks for at most @p budget.
        void _spin(std::chrono::milliseconds budget);

        /// @brief Cancels the goal in flight, if any, and forgets it.
        void _cancel();

        rclcpp::Node::SharedPtr _node;
        rclcpp::CallbackGroup::SharedPtr _callback_group;
        rclcpp::executors::SingleThreadedExecutor _executor;
        rclcpp_action::Client<FollowJointTrajectory>::SharedPtr _client;
        std::string _action_name;

        // Per goal. The callbacks that write these run inside _spin(), on the tree's
        // own thread, so they need no locking.
        bool _goal_responded{false};
        GoalHandle::SharedPtr _goal_handle;
        std::optional<GoalHandle::WrappedResult> _result;
        std::chrono::steady_clock::time_point _deadline;
    };

}  // namespace mission_bt_server
