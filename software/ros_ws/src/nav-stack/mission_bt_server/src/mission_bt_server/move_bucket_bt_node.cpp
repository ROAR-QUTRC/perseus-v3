/// @file move_bucket_bt_node.cpp
/// @brief Implementation and BT.CPP plugin registration for MoveBucketBtNode.

#include "mission_bt_server/mission_bt_server/move_bucket_bt_node.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

namespace mission_bt_server
{

    namespace
    {
        const char* const DEFAULT_ACTION =
            "/payloads/bucket_trajectory_controller/follow_joint_trajectory";

        // How long the first tick waits for the controller's action server.
        constexpr auto SERVER_WAIT = std::chrono::seconds(2);
        // Past the move's own duration, how long to wait for the controller's verdict
        // before giving up. Its goal_time tolerance is 3 s, so it always answers
        // within that on its own; this only catches a controller that went away.
        constexpr auto RESULT_GRACE = std::chrono::seconds(10);

        double radians(double degrees)
        {
            return degrees * M_PI / 180.0;
        }
    }  // namespace

    MoveBucketBtNode::MoveBucketBtNode(const std::string& name,
                                       const BT::NodeConfiguration& conf)
        : BT::StatefulActionNode(name, conf)
    {
        _node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        _callback_group =
            _node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        _executor.add_callback_group(_callback_group, _node->get_node_base_interface());
    }

    BT::PortsList MoveBucketBtNode::providedPorts()
    {
        return {
            BT::InputPort<double>("lift_deg", "bucket_lift_joint target, degrees"),
            BT::InputPort<double>("tilt_deg", "bucket_tilt_joint target, degrees"),
            BT::InputPort<double>("jaw_deg", "bucket_jaw_joint target, degrees"),
            BT::InputPort<double>("duration_s", 10.0,
                                  "time_from_start of the goal: how long the move takes"),
            BT::InputPort<std::string>("server_name", DEFAULT_ACTION,
                                       "the bucket controller's FollowJointTrajectory action"),
        };
    }

    void MoveBucketBtNode::_make_client(const std::string& action_name)
    {
        if (_client && action_name == _action_name)
        {
            return;
        }
        _action_name = action_name;
        _client = rclcpp_action::create_client<FollowJointTrajectory>(_node, action_name,
                                                                      _callback_group);
    }

    void MoveBucketBtNode::_spin(std::chrono::milliseconds budget)
    {
        _executor.spin_some(budget);
    }

    BT::NodeStatus MoveBucketBtNode::onStart()
    {
        double lift = 0.0;
        double tilt = 0.0;
        double jaw = 0.0;
        double duration_s = 10.0;
        std::string action_name = DEFAULT_ACTION;
        if (!getInput("lift_deg", lift) || !getInput("tilt_deg", tilt) ||
            !getInput("jaw_deg", jaw))
        {
            RCLCPP_ERROR(_node->get_logger(),
                         "MoveBucket: lift_deg, tilt_deg and jaw_deg are all required");
            return BT::NodeStatus::FAILURE;
        }
        getInput("duration_s", duration_s);
        getInput("server_name", action_name);
        duration_s = std::max(duration_s, 0.5);

        _make_client(action_name);
        if (!_client->wait_for_action_server(SERVER_WAIT))
        {
            RCLCPP_ERROR(_node->get_logger(),
                         "MoveBucket: %s is not available - is the bucket controller "
                         "running (payload:=bucket, bucket_controller not none)?",
                         action_name.c_str());
            return BT::NodeStatus::FAILURE;
        }

        FollowJointTrajectory::Goal goal;
        goal.trajectory.joint_names = {"bucket_lift_joint", "bucket_tilt_joint",
                                       "bucket_jaw_joint"};
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {radians(lift), radians(tilt), radians(jaw)};
        point.time_from_start = rclcpp::Duration::from_seconds(duration_s);
        goal.trajectory.points.push_back(point);

        _goal_responded = false;
        _goal_handle.reset();
        _result.reset();
        rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;
        options.goal_response_callback = [this](GoalHandle::SharedPtr handle)
        {
            _goal_responded = true;
            _goal_handle = handle;
        };
        options.result_callback = [this](const GoalHandle::WrappedResult& result)
        { _result = result; };
        _client->async_send_goal(goal, options);
        _deadline = std::chrono::steady_clock::now() +
                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double>(duration_s)) +
                    RESULT_GRACE;

        RCLCPP_INFO(_node->get_logger(),
                    "MoveBucket: lift %.1f, tilt %.1f, jaw %.1f deg over %.1f s", lift, tilt,
                    jaw, duration_s);
        _spin(std::chrono::milliseconds(5));
        return onRunning();
    }

    BT::NodeStatus MoveBucketBtNode::onRunning()
    {
        _spin(std::chrono::milliseconds(5));

        if (_goal_responded && !_goal_handle)
        {
            RCLCPP_ERROR(_node->get_logger(), "MoveBucket: %s rejected the goal",
                         _action_name.c_str());
            return BT::NodeStatus::FAILURE;
        }
        if (_result)
        {
            const auto result = *_result;
            _goal_handle.reset();
            _result.reset();
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
                result.result->error_code ==
                    FollowJointTrajectory::Result::SUCCESSFUL)
            {
                return BT::NodeStatus::SUCCESS;
            }
            RCLCPP_ERROR(_node->get_logger(), "MoveBucket: the bucket did not reach the pose: %s",
                         result.result ? result.result->error_string.c_str() : "no result");
            return BT::NodeStatus::FAILURE;
        }
        if (std::chrono::steady_clock::now() > _deadline)
        {
            RCLCPP_ERROR(_node->get_logger(), "MoveBucket: no result from %s in time",
                         _action_name.c_str());
            _cancel();
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    void MoveBucketBtNode::onHalted()
    {
        _cancel();
    }

    void MoveBucketBtNode::_cancel()
    {
        if (_goal_handle && !_result)
        {
            _client->async_cancel_goal(_goal_handle);
            // Lets the cancel request go out before the tree moves on.
            _spin(std::chrono::milliseconds(50));
            RCLCPP_INFO(_node->get_logger(), "MoveBucket: cancelled - bucket holds here");
        }
        _goal_handle.reset();
        _result.reset();
    }

}  // namespace mission_bt_server

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<mission_bt_server::MoveBucketBtNode>("MoveBucket");
}
