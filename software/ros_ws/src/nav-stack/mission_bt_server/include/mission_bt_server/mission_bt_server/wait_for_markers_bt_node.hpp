#pragma once

/// @file wait_for_markers_bt_node.hpp
/// @brief BT.CPP leaf that waits until the camera sees enough ArUco markers.

#include <behaviortree_cpp/action_node.h>

#include <interfaces/msg/detection_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace mission_bt_server
{

    /// @brief RUNNING until one frame of /vision/aruco/detections holds at least
    /// min_markers distinct markers with a 3D pose, then SUCCESS. Never fails by itself:
    /// it is meant to sit beside a Spin in a Parallel (mission.xml's ScanAndLocalise),
    /// where its success halts the Spin and stops the rover.
    ///
    /// It counts markers, not surveyed markers: it cannot tell the rail's from a stray
    /// one. That is fine, because it only decides when to stop. The fix is then taken
    /// by LocaliseInArena, which does check them, and a stop on a stray marker simply
    /// fails that and the scan carries on.
    ///
    /// Subscribes only while it is running, and only counts frames from after it
    /// started. Runs its own executor on its own callback group, like MoveBucket.
    class WaitForMarkersBtNode : public BT::StatefulActionNode
    {
    public:
        WaitForMarkersBtNode(const std::string& name, const BT::NodeConfiguration& conf);

        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;
        BT::NodeStatus onRunning() override;
        void onHalted() override;

    private:
        void _on_detections(interfaces::msg::DetectionArray::ConstSharedPtr msg);

        rclcpp::Node::SharedPtr _node;
        rclcpp::CallbackGroup::SharedPtr _callback_group;
        rclcpp::executors::SingleThreadedExecutor _executor;
        rclcpp::Subscription<interfaces::msg::DetectionArray>::SharedPtr _subscription;
        std::string _topic;
        int _min_markers{2};
        bool _seen{false};
    };

}  // namespace mission_bt_server
