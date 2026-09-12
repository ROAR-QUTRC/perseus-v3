/// @file request_zone_waypoint_bt_node.cpp
/// @brief Implementation and BT.CPP plugin registration for
/// RequestZoneWaypointBtNode.

#include "mission_bt_server/mission_bt_server/request_zone_waypoint_bt_node.hpp"

#include <behaviortree_cpp/bt_factory.h>

namespace mission_bt_server {

RequestZoneWaypointBtNode::RequestZoneWaypointBtNode(
    const std::string &service_node_name, const BT::NodeConfiguration &conf)
    : nav2_behavior_tree::BtServiceNode<interfaces::srv::RequestZoneWaypoint>(
          service_node_name, conf) {}

BT::PortsList RequestZoneWaypointBtNode::providedPorts() {
  return providedBasicPorts({
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
          "goal", "Safe point returned by the service, in the arena (map) frame"),
  });
}

BT::NodeStatus RequestZoneWaypointBtNode::on_completion(
    std::shared_ptr<interfaces::srv::RequestZoneWaypoint::Response> response) {
  if (!response->success) {
    RCLCPP_WARN(node_->get_logger(), "%s: %s", service_name_.c_str(),
               response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }
  setOutput("goal", response->waypoint);
  return BT::NodeStatus::SUCCESS;
}

} // namespace mission_bt_server

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<mission_bt_server::RequestZoneWaypointBtNode>(
      "RequestZoneWaypoint");
}
