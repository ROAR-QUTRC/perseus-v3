/// @file blind_jaw_bt_node.cpp
/// @brief Implementation and BT.CPP plugin registration for BlindJawBtNode.

#include "mission_bt_server/mission_bt_server/blind_jaw_bt_node.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <algorithm>
#include <cmath>
#include <functional>

namespace mission_bt_server
{

    namespace
    {
        // How long to wait for the driver to take the bucket from autonomy.
        constexpr auto TAKEOVER_TIMEOUT = std::chrono::seconds(2);
        // Zeros before the rearm, so the driver's latest speed is below its deadband.
        constexpr auto SETTLE = std::chrono::milliseconds(400);
        constexpr auto REARM_TIMEOUT = std::chrono::seconds(5);
        // The driver zeros its speeds if no command arrives for a short while, so
        // publish well inside that.
        constexpr auto PUBLISH_PERIOD = std::chrono::milliseconds(40);
        // Actuators velocity order: lift, tilt, jaws.
        constexpr size_t JAW_INDEX = 2;
    }  // namespace

    BlindJawBtNode::BlindJawBtNode(const std::string& name, const BT::NodeConfiguration& conf)
        : BT::StatefulActionNode(name, conf)
    {
        _node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        _callback_group =
            _node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        _executor.add_callback_group(_callback_group, _node->get_node_base_interface());

        _publisher = _node->create_publisher<actuator_msgs::msg::Actuators>("bucket_actuators", 10);
        _rearm_client =
            _node->create_client<std_srvs::srv::Trigger>("/bucket/rearm", rclcpp::ServicesQoS(),
                                                         _callback_group);

        rclcpp::SubscriptionOptions options;
        options.callback_group = _callback_group;
        _override_subscription = _node->create_subscription<std_msgs::msg::Bool>(
            "/bucket/operator_override", rclcpp::QoS(1).reliable().transient_local(),
            [this](const std_msgs::msg::Bool& msg)
            { _operator_override = msg.data; }, options);
    }

    BT::PortsList BlindJawBtNode::providedPorts()
    {
        return {
            BT::InputPort<double>("speed", 0.0,
                                  "jaw velocity in bucket_actuators units; the sign is the "
                                  "direction, 0 refuses to move"),
            BT::InputPort<double>("duration_s", 0.0, "how long to drive the jaw, seconds"),
            BT::InputPort<double>("max_s", 8.0,
                                  "the longest a move may last; duration_s is clamped to it, "
                                  "because a jaw driven into its end stop stalls the motor"),
        };
    }

    void BlindJawBtNode::_spin(std::chrono::milliseconds budget)
    {
        _executor.spin_some(budget);
    }

    void BlindJawBtNode::_publish(double jaw_speed)
    {
        actuator_msgs::msg::Actuators message;
        message.header.stamp = _node->now();
        message.velocity.assign(3, 0.0);
        message.velocity[JAW_INDEX] = jaw_speed;
        _publisher->publish(message);
        _last_publish = std::chrono::steady_clock::now();
    }

    void BlindJawBtNode::_stop()
    {
        // A few, so one dropped message cannot leave the last speed standing.
        for (int i = 0; i < 3; ++i)
        {
            _publish(0.0);
            _spin(std::chrono::milliseconds(5));
        }
    }

    BT::NodeStatus BlindJawBtNode::onStart()
    {
        getInput("speed", _speed);
        getInput("duration_s", _duration_s);
        double max_s = 0.0;
        getInput("max_s", max_s);

        if (!std::isfinite(_speed) || !std::isfinite(_duration_s) || _speed == 0.0 ||
            _duration_s <= 0.0 || !std::isfinite(max_s) || max_s <= 0.0)
        {
            RCLCPP_ERROR(_node->get_logger(),
                         "BlindJawMove: speed %.3f, duration %.2f s and cap %.2f s do not "
                         "describe a move - the blind jaw values have not been set "
                         "(jaw_blind_* parameters on mission_bt_server), so the jaw is left "
                         "where it is",
                         _speed, _duration_s, max_s);
            return BT::NodeStatus::FAILURE;
        }
        _duration_s = std::min(_duration_s, max_s);

        _spin(std::chrono::milliseconds(5));  // the latched override flag
        if (_operator_override)
        {
            RCLCPP_ERROR(_node->get_logger(),
                         "BlindJawMove: the operator has the bucket - hand it back with "
                         "`ros2 run payloads bucket_supervisor.py --rearm` first");
            return BT::NodeStatus::FAILURE;
        }
        if (!_rearm_client->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(_node->get_logger(),
                         "BlindJawMove: /bucket/rearm is not available - is bucket_driver "
                         "running? Not moving the jaw, since it could not be handed back");
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_WARN(_node->get_logger(),
                    "BlindJawMove: jaw speed %+.3f for %.1f s with no position feedback",
                    _speed, _duration_s);
        _phase = Phase::TAKEOVER;
        _phase_start = std::chrono::steady_clock::now();
        _rearm_sent = _rearm_answered = _rearm_ok = false;
        _publish(_speed);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus BlindJawBtNode::onRunning()
    {
        _spin(std::chrono::milliseconds(5));
        const auto now = std::chrono::steady_clock::now();
        const auto since_publish = now - _last_publish;

        switch (_phase)
        {
        case Phase::TAKEOVER:
            if (_operator_override)
            {
                _phase = Phase::DRIVING;
                _phase_start = now;
                RCLCPP_INFO(_node->get_logger(), "BlindJawMove: driver has the bucket, moving");
            }
            else if (now - _phase_start > TAKEOVER_TIMEOUT)
            {
                RCLCPP_ERROR(_node->get_logger(),
                             "BlindJawMove: bucket_driver did not take the bucket in 2 s - "
                             "stopping");
                _stop();
                return BT::NodeStatus::FAILURE;
            }
            if (since_publish >= PUBLISH_PERIOD)
                _publish(_speed);
            return BT::NodeStatus::RUNNING;

        case Phase::DRIVING:
            if (now - _phase_start >= std::chrono::duration<double>(_duration_s))
            {
                _phase = Phase::SETTLE;
                _phase_start = now;
                _publish(0.0);
                return BT::NodeStatus::RUNNING;
            }
            if (since_publish >= PUBLISH_PERIOD)
                _publish(_speed);
            return BT::NodeStatus::RUNNING;

        case Phase::SETTLE:
            if (since_publish >= PUBLISH_PERIOD)
                _publish(0.0);
            if (now - _phase_start >= SETTLE)
            {
                _phase = Phase::REARM;
                _phase_start = now;
            }
            return BT::NodeStatus::RUNNING;

        case Phase::REARM:
            if (since_publish >= PUBLISH_PERIOD)
                _publish(0.0);
            if (!_rearm_sent)
            {
                _rearm_sent = true;
                _rearm_client->async_send_request(
                    std::make_shared<std_srvs::srv::Trigger::Request>(),
                    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
                    {
                        const auto response = future.get();
                        _rearm_ok = response->success;
                        _rearm_message = response->message;
                        _rearm_answered = true;
                    });
                return BT::NodeStatus::RUNNING;
            }
            if (_rearm_answered)
            {
                if (_rearm_ok)
                {
                    RCLCPP_INFO(_node->get_logger(),
                                "BlindJawMove: done, %s - jaw position is not verified",
                                _rearm_message.c_str());
                    return BT::NodeStatus::SUCCESS;
                }
                RCLCPP_ERROR(_node->get_logger(),
                             "BlindJawMove: the jaw moved but the bucket could not be handed "
                             "back: %s",
                             _rearm_message.c_str());
                return BT::NodeStatus::FAILURE;
            }
            if (now - _phase_start > REARM_TIMEOUT)
            {
                RCLCPP_ERROR(_node->get_logger(),
                             "BlindJawMove: no answer from /bucket/rearm in 5 s");
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::RUNNING;
        }
        return BT::NodeStatus::FAILURE;
    }

    void BlindJawBtNode::onHalted()
    {
        // The mission was stopped mid-move: the jaw must not keep driving. The bucket
        // stays with the driver; a person re-arms it.
        _stop();
        RCLCPP_WARN(_node->get_logger(), "BlindJawMove: halted - jaw stopped");
    }

}  // namespace mission_bt_server

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<mission_bt_server::BlindJawBtNode>("BlindJawMove");
}
