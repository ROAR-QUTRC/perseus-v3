/// @file wiggle_bucket_bt_node.cpp
/// @brief Implementation of WiggleBucketBtNode. It is registered with MoveBucket, in
/// move_bucket_bt_node.cpp, as the two share one plugin library.

#include "mission_bt_server/mission_bt_server/wiggle_bucket_bt_node.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>

#include "mission_bt_server/mission_bt_server/bucket_mission_params.hpp"

namespace mission_bt_server
{

    namespace
    {
        // How long onStart waits for a fresh /joint_states (20 Hz on the rover).
        constexpr auto READING_WAIT = std::chrono::seconds(1);
        // At most how long onStart spends discarding readings queued while the node
        // sat idle, before it waits for a fresh one.
        constexpr auto DRAIN_LIMIT = std::chrono::milliseconds(200);

        struct Axis
        {
            const char* port;
            const char* joint;
            const char* label;
        };
        constexpr std::array<Axis, 3> AXES{{{"lift_deg", "bucket_lift_joint", "lift"},
                                            {"tilt_deg", "bucket_tilt_joint", "tilt"},
                                            {"jaw_deg", "bucket_jaw_joint", "jaw"}}};

        double radians(double degrees)
        {
            return degrees * M_PI / 180.0;
        }

        double degrees(double radians)
        {
            return radians * 180.0 / M_PI;
        }
    }  // namespace

    WiggleBucketBtNode::WiggleBucketBtNode(const std::string& name,
                                           const BT::NodeConfiguration& conf)
        : MoveBucketBtNode(name, conf)
    {
        // The linkage broadcaster also publishes its ram joints on /joint_states, so
        // pick the three bucket joints out by name.
        rclcpp::SubscriptionOptions options;
        options.callback_group = _callback_group;
        _joint_subscription = _node->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::QoS(10),
            [this](const sensor_msgs::msg::JointState& msg)
            {
                ++_joint_messages;
                for (std::size_t i = 0; i < msg.name.size() && i < msg.position.size(); ++i)
                {
                    for (std::size_t axis = 0; axis < AXES.size(); ++axis)
                    {
                        if (msg.name[i] == AXES[axis].joint)
                        {
                            _angles[axis] = msg.position[i];
                        }
                    }
                }
            },
            options);
    }

    BT::PortsList WiggleBucketBtNode::providedPorts()
    {
        return {
            BT::InputPort<double>("lift_deg", 5.0,
                                  "lift offset, degrees: + lowers first, - raises; 0 skips"),
            BT::InputPort<double>("tilt_deg", 5.0,
                                  "tilt offset, degrees: + dumps first, - curls; 0 skips"),
            BT::InputPort<double>("jaw_deg", 5.0,
                                  "jaw offset, degrees: + opens first, - closes; 0 skips"),
            BT::InputPort<double>("timeout_s", 3.0, "how long each move may take"),
            BT::InputPort<double>("tolerance_deg", 2.0,
                                  "how close counts as there, for every move; always sent, "
                                  "as the controller's own tilt tolerance is ~29 deg"),
            BT::InputPort<std::string>("server_name", DEFAULT_BUCKET_ACTION,
                                       "the bucket controller's FollowJointTrajectory action"),
            BT::OutputPort<std::string>(
                "failure",
                "why the wiggle failed, empty on success: invalid, no_joint_state, "
                "operator_override, no_server or rejected. A missed move is not a failure"),
        };
    }

    BT::NodeStatus WiggleBucketBtNode::onStart()
    {
        setOutput("failure", std::string{});
        std::array<double, 3> offsets{};
        bool any = false;
        for (std::size_t axis = 0; axis < AXES.size(); ++axis)
        {
            getInput(AXES[axis].port, offsets[axis]);
            any = any || offsets[axis] != 0.0;
        }
        if (!any)
        {
            RCLCPP_ERROR(_node->get_logger(),
                         "WiggleBucket: lift_deg, tilt_deg and jaw_deg are all 0");
            setOutput("failure", std::string("invalid"));
            return BT::NodeStatus::FAILURE;
        }
        std::string action_name = DEFAULT_BUCKET_ACTION;
        getInput("server_name", action_name);
        double timeout_s = 3.0;
        getInput("timeout_s", timeout_s);
        double tolerance_deg = 2.0;
        getInput("tolerance_deg", tolerance_deg);

        // Readings queued while the tree was elsewhere can be from long ago - the
        // encoders may since have stopped reaching /joint_states. Throw them away,
        // then wait for new ones.
        const auto drain_until = std::chrono::steady_clock::now() + DRAIN_LIMIT;
        std::size_t seen = 0;
        do
        {
            seen = _joint_messages;
            _spin(std::chrono::milliseconds(2));
        } while (_joint_messages != seen && std::chrono::steady_clock::now() < drain_until);
        _angles = {};
        const auto missing = [&]
        {
            std::string names;
            for (std::size_t axis = 0; axis < AXES.size(); ++axis)
            {
                if (offsets[axis] != 0.0 && !_angles[axis])
                {
                    names += names.empty() ? AXES[axis].joint
                                           : std::string(", ") + AXES[axis].joint;
                }
            }
            return names;
        };
        const auto wait_until = std::chrono::steady_clock::now() + READING_WAIT;
        while (!missing().empty() && std::chrono::steady_clock::now() < wait_until)
        {
            _spin(std::chrono::milliseconds(10));
        }
        if (const auto names = missing(); !names.empty())
        {
            RCLCPP_ERROR(_node->get_logger(),
                         "WiggleBucket: no fresh reading of %s on /joint_states - is the "
                         "bucket_joint_state_broadcaster running?",
                         names.c_str());
            setOutput("failure", std::string("no_joint_state"));
            return BT::NodeStatus::FAILURE;
        }

        std::vector<Step> steps;
        for (std::size_t axis = 0; axis < AXES.size(); ++axis)
        {
            if (offsets[axis] == 0.0)
            {
                continue;
            }
            const double start = *_angles[axis];
            const double out = start + radians(offsets[axis]);
            char text[48];
            std::snprintf(text, sizeof(text), "wiggle %s to %.1f", AXES[axis].label,
                          degrees(out));
            steps.push_back(
                _direct_step({AXES[axis].joint}, {out}, timeout_s, tolerance_deg, text));
            std::snprintf(text, sizeof(text), "wiggle %s back to %.1f", AXES[axis].label,
                          degrees(start));
            steps.push_back(
                _direct_step({AXES[axis].joint}, {start}, timeout_s, tolerance_deg, text));
        }
        return _start(action_name, std::move(steps), 1, true);
    }

}  // namespace mission_bt_server
