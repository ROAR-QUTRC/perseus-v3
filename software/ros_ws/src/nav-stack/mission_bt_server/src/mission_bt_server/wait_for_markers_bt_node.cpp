/// @file wait_for_markers_bt_node.cpp
/// @brief Implementation and BT.CPP plugin registration for WaitForMarkersBtNode.

#include "mission_bt_server/mission_bt_server/wait_for_markers_bt_node.hpp"

#include <behaviortree_cpp/bt_factory.h>

#include <chrono>
#include <set>
#include <string>

namespace mission_bt_server
{

    namespace
    {
        constexpr auto SPIN_BUDGET = std::chrono::milliseconds(10);
    }

    WaitForMarkersBtNode::WaitForMarkersBtNode(const std::string& name,
                                               const BT::NodeConfiguration& conf)
        : BT::StatefulActionNode(name, conf)
    {
        _node = config().blackboard->get<rclcpp::Node::SharedPtr>("@node");
        _callback_group =
            _node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
        _executor.add_callback_group(_callback_group, _node->get_node_base_interface());
    }

    BT::PortsList WaitForMarkersBtNode::providedPorts()
    {
        return {
            BT::InputPort<int>("min_markers", 2, "markers with a 3D pose needed in one frame"),
            BT::InputPort<std::string>("topic", "/vision/aruco/detections",
                                       "the ArUco detector's DetectionArray topic"),
        };
    }

    void WaitForMarkersBtNode::_on_detections(
        interfaces::msg::DetectionArray::ConstSharedPtr msg)
    {
        std::set<int> ids;
        for (const auto& detection : msg->detections)
        {
            if (detection.has_pose)
            {
                ids.insert(detection.id);
            }
        }
        if (static_cast<int>(ids.size()) >= _min_markers)
        {
            _seen = true;
        }
    }

    BT::NodeStatus WaitForMarkersBtNode::onStart()
    {
        getInput("min_markers", _min_markers);
        getInput("topic", _topic);
        _seen = false;

        rclcpp::SubscriptionOptions options;
        options.callback_group = _callback_group;
        _subscription = _node->create_subscription<interfaces::msg::DetectionArray>(
            _topic, rclcpp::SensorDataQoS(),
            [this](interfaces::msg::DetectionArray::ConstSharedPtr msg)
            { _on_detections(std::move(msg)); },
            options);
        return onRunning();
    }

    BT::NodeStatus WaitForMarkersBtNode::onRunning()
    {
        _executor.spin_some(SPIN_BUDGET);
        if (!_seen)
        {
            return BT::NodeStatus::RUNNING;
        }
        _subscription.reset();
        RCLCPP_INFO(_node->get_logger(), "WaitForMarkers: %d or more markers in view",
                    _min_markers);
        return BT::NodeStatus::SUCCESS;
    }

    void WaitForMarkersBtNode::onHalted()
    {
        _subscription.reset();
    }

}  // namespace mission_bt_server

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<mission_bt_server::WaitForMarkersBtNode>("WaitForMarkers");
}
