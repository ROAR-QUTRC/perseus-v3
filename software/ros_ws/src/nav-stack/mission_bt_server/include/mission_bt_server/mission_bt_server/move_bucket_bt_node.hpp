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
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <vector>

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
    /// are, so <MoveBucket jaw_deg="36"/> opens the jaw and nothing else.
    ///
    /// Two ways to move, picked by whether duration_s is given:
    ///
    ///   direct (no duration_s, the usual one): the setpoint jumps to the pose at
    ///   once and the board drives there at its own speed. The board's position mode
    ///   is full duty until it is within 2 deg, so this is one continuous motion. The
    ///   goal carries timeout_s as its goal_time tolerance: the controller succeeds as
    ///   soon as the bucket is within tolerance and gives up once timeout_s is out.
    ///
    ///   timed (duration_s): one point reached duration_s after it is sent, which the
    ///   controller interpolates toward. The board chases a creeping setpoint in
    ///   full-duty bursts, so a long one judders; kept for moves that must keep pace
    ///   with something else, like DigBucket's carry alongside its drive.
    ///
    /// A pose that is not reached (not_reached, or no result in time) is sent again,
    /// up to attempts in all. Nothing else is retried: an operator override, a missing
    /// or refusing controller fail at once.
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
    ///
    /// While the operator has the bucket (/bucket/operator_override, latched by
    /// payloads' bucket_driver) it fails at once, and a move already running fails
    /// the moment the operator takes over, rather than waiting out the controller.
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

    protected:
        /// @brief One goal of a move. A move is one or more of these, sent in order,
        /// each only once the one before it has finished.
        struct Step
        {
            FollowJointTrajectory::Goal goal;
            /// For the log: "lift 12.0 deg within 5.0 s".
            std::string described;
            /// How long to wait for the controller's verdict on this goal.
            std::chrono::steady_clock::duration allowance{};
        };

        /// @brief A direct move of @p joints to @p positions (radians): the setpoint
        /// jumps there and the controller gives the bucket @p timeout_s to arrive.
        /// @p tolerance_deg, if above 0, replaces the controller's goal tolerance for
        /// each joint. @p described is the joints and angles, for the log.
        static Step _direct_step(const std::vector<std::string>& joints,
                                 const std::vector<double>& positions, double timeout_s,
                                 std::optional<double> tolerance_deg, std::string described);

        /// @brief Runs @p steps in order on @p action_name, each sent up to
        /// @p attempts times. With @p tolerate_miss a step that is still missed after
        /// its last attempt is logged and skipped rather than failing the move.
        /// Called from onStart(); returns what onStart() should.
        BT::NodeStatus _start(const std::string& action_name, std::vector<Step> steps,
                              int attempts, bool tolerate_miss);

        /// @brief Services the action client's callbacks for at most @p budget.
        void _spin(std::chrono::milliseconds budget);

        rclcpp::Node::SharedPtr _node;
        rclcpp::CallbackGroup::SharedPtr _callback_group;

    private:
        /// @brief Creates the action client for @p action_name, replacing one made
        /// for a different name.
        void _make_client(const std::string& action_name);

        /// @brief Cancels the goal in flight, if any, and forgets it.
        void _cancel();

        /// @brief Sends the current step's goal and starts this attempt's clock.
        void _send_goal();

        /// @brief After a missed pose: sends it again and returns RUNNING if attempts
        /// are left, else records @p failure and returns FAILURE - or, with
        /// _tolerate_miss, moves on to the next step.
        BT::NodeStatus _retry_or_fail(const char* failure);

        /// @brief Sends the next step and returns RUNNING, or SUCCESS after the last.
        BT::NodeStatus _next_step();

        /// @brief How far each joint was from the current step's pose in the
        /// controller's last feedback: "jaw 3.3 deg off: target 36.2, actual 32.9,
        /// tolerance 2.0". The controller's own result only says the goal timed out,
        /// and which joint missed by how much is otherwise only in its log.
        std::string _describe_miss() const;

        rclcpp::executors::SingleThreadedExecutor _executor;
        rclcpp_action::Client<FollowJointTrajectory>::SharedPtr _client;
        std::string _action_name;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _override_subscription;
        bool _operator_override{false};

        // Per tick of onStart: the steps, each goal resent as is on each attempt.
        std::vector<Step> _steps;
        std::size_t _step{0};
        bool _tolerate_miss{false};
        int _attempt{0};
        int _attempts{1};

        // Per goal. The callbacks that write these run inside _spin(), on the tree's
        // own thread, so they need no locking.
        bool _goal_responded{false};
        GoalHandle::SharedPtr _goal_handle;
        std::optional<GoalHandle::WrappedResult> _result;
        std::shared_ptr<const FollowJointTrajectory::Feedback> _feedback;
        std::chrono::steady_clock::time_point _deadline;
    };

}  // namespace mission_bt_server
