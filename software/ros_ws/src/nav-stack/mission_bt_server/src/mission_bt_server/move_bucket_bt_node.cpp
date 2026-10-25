/// @file move_bucket_bt_node.cpp
/// @brief Implementation and BT.CPP plugin registration for MoveBucketBtNode.

#include "mission_bt_server/mission_bt_server/move_bucket_bt_node.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <tuple>
#include <vector>

#include "mission_bt_server/mission_bt_server/bucket_mission_params.hpp"

namespace mission_bt_server
{

    namespace
    {
        const char* const DEFAULT_ACTION = DEFAULT_BUCKET_ACTION;

        // How long the first tick waits for the controller's action server.
        constexpr auto SERVER_WAIT = std::chrono::seconds(2);
        // Past the move's own allowance (duration_s plus the controller's 3 s
        // goal_time, or timeout_s), how long to wait for the controller's verdict
        // before giving up. It always answers within the allowance on its own; this
        // only catches a controller that went away.
        constexpr auto RESULT_GRACE = std::chrono::seconds(10);
        // The yaml's goal_time, which a timed move's goal leaves in force.
        constexpr double CONTROLLER_GOAL_TIME_S = 3.0;
        // A direct move's one point: as good as now, so the setpoint jumps to the
        // pose, but not zero, which the controller would read as "already there".
        constexpr double DIRECT_POINT_S = 0.1;
        // A takeover deactivates the controller, which aborts the goal a moment before
        // bucket_driver latches the override flag. Before retrying a missed pose, wait
        // this long for that flag, so a takeover is never mistaken for a miss.
        constexpr auto OVERRIDE_SETTLE = std::chrono::milliseconds(300);

        double radians(double degrees)
        {
            return degrees * M_PI / 180.0;
        }
    }  // namespace

    MoveBucketBtNode::MoveBucketBtNode(const std::string& name,
                                       const BT::NodeConfiguration& conf)
        : BT::StatefulActionNode(name, conf)
    {
        // From the root blackboard, so this works inside a subtree that was not
        // remapped - a saved pose dropped into a tree in Groot2, say.
        _node = config().blackboard->get<rclcpp::Node::SharedPtr>("@node");
        _callback_group =
            _node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        _executor.add_callback_group(_callback_group, _node->get_node_base_interface());

        rclcpp::SubscriptionOptions options;
        options.callback_group = _callback_group;
        _override_subscription = _node->create_subscription<std_msgs::msg::Bool>(
            "/bucket/operator_override", rclcpp::QoS(1).reliable().transient_local(),
            [this](const std_msgs::msg::Bool& msg)
            { _operator_override = msg.data; }, options);
    }

    BT::PortsList MoveBucketBtNode::providedPorts()
    {
        return {
            BT::InputPort<double>("lift_deg", "bucket_lift_joint target, degrees; omit to hold"),
            BT::InputPort<double>("tilt_deg", "bucket_tilt_joint target, degrees; omit to hold"),
            BT::InputPort<double>("jaw_deg", "bucket_jaw_joint target, degrees; omit to hold"),
            BT::InputPort<double>("timeout_s", 10.0,
                                  "direct move: how long the bucket may take to arrive"),
            BT::InputPort<int>("attempts", 1,
                               "how many times to send the pose if it is not reached"),
            BT::InputPort<double>("tolerance_deg",
                                  "how close counts as there; omit for the controller's 2 deg"),
            BT::InputPort<double>("duration_s",
                                  "timed move instead: the controller interpolates over this "
                                  "long; omit for a direct move"),
            BT::InputPort<std::string>("server_name", DEFAULT_ACTION,
                                       "the bucket controller's FollowJointTrajectory action"),
            BT::OutputPort<std::string>(
                "failure",
                "why the move failed, empty on success: invalid, operator_override, no_server, "
                "rejected, not_reached (the controller ran but the pose was missed) or timeout"),
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
        // Only the joints given a port are commanded; the controller holds the rest
        // where they are (allow_partial_joints_goal in bucket_controller.yaml).
        setOutput("failure", std::string{});
        std::vector<std::string> joints;
        std::vector<double> positions;
        _described.clear();
        for (const auto& [port, joint, label] :
             {std::tuple{"lift_deg", "bucket_lift_joint", "lift"},
              std::tuple{"tilt_deg", "bucket_tilt_joint", "tilt"},
              std::tuple{"jaw_deg", "bucket_jaw_joint", "jaw"}})
        {
            double degrees = 0.0;
            if (getInput(port, degrees))
            {
                joints.emplace_back(joint);
                positions.push_back(radians(degrees));
                char text[32];
                std::snprintf(text, sizeof(text), "%s%s %.1f", _described.empty() ? "" : ", ",
                              label, degrees);
                _described += text;
            }
        }
        if (joints.empty())
        {
            RCLCPP_ERROR(_node->get_logger(),
                         "MoveBucket: give at least one of lift_deg, tilt_deg, jaw_deg");
            setOutput("failure", std::string("invalid"));
            return BT::NodeStatus::FAILURE;
        }
        std::string action_name = DEFAULT_ACTION;
        getInput("server_name", action_name);
        double timeout_s = 10.0;
        getInput("timeout_s", timeout_s);
        _attempts = 1;
        getInput("attempts", _attempts);
        _attempts = std::max(_attempts, 1);
        const auto duration_s = getInput<double>("duration_s");
        const auto tolerance_deg = getInput<double>("tolerance_deg");

        _spin(std::chrono::milliseconds(5));  // the latched override flag
        if (_operator_override)
        {
            RCLCPP_ERROR(_node->get_logger(),
                         "MoveBucket: the operator has the bucket - hand it back with "
                         "`a` in teleop_console, or `ros2 run payloads "
                         "bucket_supervisor.py --rearm`, first");
            setOutput("failure", std::string("operator_override"));
            return BT::NodeStatus::FAILURE;
        }

        _make_client(action_name);
        if (!_client->wait_for_action_server(SERVER_WAIT))
        {
            RCLCPP_ERROR(_node->get_logger(),
                         "MoveBucket: %s is not available - is the bucket controller "
                         "running (perseus.launch.py payload:=bucket, bucket_controller "
                         "not none)?",
                         action_name.c_str());
            setOutput("failure", std::string("no_server"));
            return BT::NodeStatus::FAILURE;
        }

        _goal = FollowJointTrajectory::Goal();
        _goal.trajectory.joint_names = joints;
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = positions;
        double allowance_s = 0.0;
        if (duration_s)
        {
            const double seconds = std::max(duration_s.value(), 0.5);
            point.time_from_start = rclcpp::Duration::from_seconds(seconds);
            allowance_s = seconds + CONTROLLER_GOAL_TIME_S;
            char text[48];
            std::snprintf(text, sizeof(text), " deg over %.1f s", seconds);
            _described += text;
        }
        else
        {
            timeout_s = std::max(timeout_s, 0.5);
            point.time_from_start = rclcpp::Duration::from_seconds(DIRECT_POINT_S);
            _goal.goal_time_tolerance = rclcpp::Duration::from_seconds(timeout_s);
            allowance_s = DIRECT_POINT_S + timeout_s;
            char text[48];
            std::snprintf(text, sizeof(text), " deg within %.1f s", timeout_s);
            _described += text;
        }
        _goal.trajectory.points.push_back(point);
        if (tolerance_deg && tolerance_deg.value() > 0.0)
        {
            for (const auto& joint : joints)
            {
                control_msgs::msg::JointTolerance tolerance;
                tolerance.name = joint;
                tolerance.position = radians(tolerance_deg.value());
                _goal.goal_tolerance.push_back(tolerance);
            }
        }
        _allowance = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                         std::chrono::duration<double>(allowance_s)) +
                     RESULT_GRACE;

        _attempt = 1;
        _send_goal();
        return onRunning();
    }

    void MoveBucketBtNode::_send_goal()
    {
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
        _client->async_send_goal(_goal, options);
        _deadline = std::chrono::steady_clock::now() + _allowance;

        if (_attempts > 1)
        {
            RCLCPP_INFO(_node->get_logger(), "MoveBucket: %s (attempt %d/%d)",
                        _described.c_str(), _attempt, _attempts);
        }
        else
        {
            RCLCPP_INFO(_node->get_logger(), "MoveBucket: %s", _described.c_str());
        }
        _spin(std::chrono::milliseconds(5));
    }

    BT::NodeStatus MoveBucketBtNode::_retry_or_fail(const char* failure)
    {
        const auto settle_until = std::chrono::steady_clock::now() + OVERRIDE_SETTLE;
        while (!_operator_override && std::chrono::steady_clock::now() < settle_until)
        {
            _spin(std::chrono::milliseconds(10));
        }
        if (_operator_override)
        {
            RCLCPP_ERROR(_node->get_logger(), "MoveBucket: operator override - stopping");
            setOutput("failure", std::string("operator_override"));
            return BT::NodeStatus::FAILURE;
        }
        if (_attempt < _attempts)
        {
            ++_attempt;
            _send_goal();
            return BT::NodeStatus::RUNNING;
        }
        setOutput("failure", std::string(failure));
        return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus MoveBucketBtNode::onRunning()
    {
        _spin(std::chrono::milliseconds(5));

        if (_operator_override)
        {
            // Taking over deactivates the controller, which aborts the goal anyway;
            // say why here rather than waiting for that.
            RCLCPP_ERROR(_node->get_logger(), "MoveBucket: operator override - stopping");
            _cancel();
            setOutput("failure", std::string("operator_override"));
            return BT::NodeStatus::FAILURE;
        }
        if (_goal_responded && !_goal_handle)
        {
            RCLCPP_ERROR(_node->get_logger(), "MoveBucket: %s rejected the goal",
                         _action_name.c_str());
            setOutput("failure", std::string("rejected"));
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
            return _retry_or_fail("not_reached");
        }
        if (std::chrono::steady_clock::now() > _deadline)
        {
            RCLCPP_ERROR(_node->get_logger(), "MoveBucket: no result from %s in time",
                         _action_name.c_str());
            _cancel();
            return _retry_or_fail("timeout");
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
