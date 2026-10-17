#pragma once

/// @file blind_jaw_bt_node.hpp
/// @brief BT.CPP leaf that drives the bucket's jaw open-loop for a set time, the
/// recovery for a jaw move whose encoder feedback cannot be trusted.

#include <behaviortree_cpp/action_node.h>

#include <actuator_msgs/msg/actuators.hpp>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>

namespace mission_bt_server
{

    /// @brief Runs the jaw at a fixed duty for duration_s, with no position feedback,
    /// then hands the bucket back to autonomy.
    ///
    /// For when MoveBucket's jaw move fails because the jaw's encoder misreads, so the
    /// controller never sees the pose it was told to reach. The jaw is then moved by
    /// time instead, like the gamepad moves it, and the mission carries on without
    /// knowing where the jaw ended up.
    ///
    /// It uses the same path as the operator: speeds go to /bucket_actuators, and
    /// payloads' bucket_driver treats any speed above its deadband as an operator
    /// override. That deactivates the bucket's trajectory controller, so the move ends
    /// by calling /bucket/rearm, which re-activates it from the encoders' current
    /// angles. The next MoveBucket then works as before - if the rearm succeeds. It is
    /// strict: with the encoders stale or a joint out of range it refuses, and this
    /// node fails with the driver's reason.
    ///
    /// Phases, one per tick of the tree:
    ///   TAKEOVER  publish the speed until the driver reports the override. The driver
    ///             only forwards speeds once it has the bucket, so the timer does not
    ///             start before that.
    ///   DRIVING   keep publishing for duration_s.
    ///   SETTLE    publish zeros, because /bucket/rearm is refused while a speed above
    ///             the deadband is still the driver's latest command.
    ///   REARM     call /bucket/rearm and wait for the answer.
    ///
    /// speed is the jaw entry of the Actuators message, signed: its sign is the
    /// direction and its size is the duty (bucket_driver's max_actuator_speed is full
    /// duty). 0 refuses to move, so nothing happens until the values have been measured
    /// on the bench. Fails at once if the operator already has the bucket.
    class BlindJawBtNode : public BT::StatefulActionNode
    {
    public:
        BlindJawBtNode(const std::string& name, const BT::NodeConfiguration& conf);

        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        enum class Phase
        {
            TAKEOVER,
            DRIVING,
            SETTLE,
            REARM
        };

        /// @brief Services the callbacks for at most @p budget.
        void _spin(std::chrono::milliseconds budget);

        /// @brief Publishes the jaw speed (lift and tilt stay 0).
        void _publish(double jaw_speed);

        /// @brief Stops the jaw and leaves the node idle.
        void _stop();

        rclcpp::Node::SharedPtr _node;
        rclcpp::CallbackGroup::SharedPtr _callback_group;
        rclcpp::executors::SingleThreadedExecutor _executor;
        rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr _publisher;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _override_subscription;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr _rearm_client;
        bool _operator_override{false};

        // Per move.
        Phase _phase{Phase::TAKEOVER};
        double _speed{0.0};
        double _duration_s{0.0};
        std::chrono::steady_clock::time_point _phase_start;
        std::chrono::steady_clock::time_point _last_publish;
        bool _rearm_sent{false};
        bool _rearm_answered{false};
        bool _rearm_ok{false};
        std::string _rearm_message;
    };

}  // namespace mission_bt_server
