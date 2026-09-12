#pragma once

/// @file mission_bt_server.hpp
/// @brief Hosts the go_to_zone_waypoint BT behind two plain services.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <nav2_behavior_tree/behavior_tree_engine.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace mission_bt_server {

/// @brief Runs go_to_zone_waypoint.xml (request a safe zone waypoint from
/// arena_server, then nav2's own NavigateToPose action) behind two services,
/// one per zone, for the RViz mission panel's two buttons to call.
///
/// Deliberately not a nav2_behavior_tree::BtActionServer<...>: that class is
/// built around a LifecycleNode and its own dedicated ROS action, which is
/// more machinery than two fire-and-forget buttons need. This node instead
/// replicates just the blackboard setup nav2_behavior_tree's own BT nodes
/// require (see BtActionServer::on_configure in bt_action_server_impl.hpp)
/// and calls BehaviorTreeEngine::run() directly inside a plain service
/// callback, matching arena_server's own _on_localise: a slow, blocking
/// handler on a reentrant callback group rather than an action server.
///
/// Two zones running "at once" is possible if both buttons are pressed close
/// together - resolved by nav2, not by this node: the second NavigateToPose
/// goal preempts the first at bt_navigator, so the first tree's action node
/// comes back CANCELLED/ABORTED and that Trigger call reports failure. No
/// separate locking is added for this; it degrades to "last press wins"
/// rather than deadlocking or corrupting anything.
class MissionBtServer : public rclcpp::Node {
public:
  MissionBtServer();

private:
  /// @brief Builds the blackboard, builds the tree from bt_xml_path with
  /// service_name pointed at one zone, and runs it to completion.
  /// @param service_name Which of arena_server's two waypoint services to
  /// call - see RequestZoneWaypointBtNode.
  /// @param[out] response Filled in with the outcome.
  void _run_mission(const std::string &service_name,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void
  _on_excavation(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void _on_construction(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  std::string _bt_xml_path;
  std::string _excavation_service_name;
  std::string _construction_service_name;

  // Blackboard entries every nav2_behavior_tree BT node reads in its own
  // constructor (BtServiceNode, BtActionNode) - see bt_action_server_impl.hpp,
  // which this replicates without the LifecycleNode it comes attached to.
  std::chrono::milliseconds _bt_loop_duration;
  std::chrono::milliseconds _server_timeout;
  std::chrono::milliseconds _cancel_timeout;
  std::chrono::milliseconds _wait_for_service_timeout;

  // A plain node distinct from `this`, purely so the BT nodes have an
  // rclcpp::Node::SharedPtr to construct clients against before `this` could
  // safely offer shared_from_this() (unavailable mid-constructor). Never
  // added to an executor: each BT node spins its own private
  // SingleThreadedExecutor bound to its own callback group when it waits on a
  // future (see BtServiceNode::check_future / BtActionNode::tick), so nothing
  // here needs to be spun globally - same pattern as bt_navigator's own
  // client_node_.
  rclcpp::Node::SharedPtr _bt_client_node;
  std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> _engine;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _excavation_srv;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _construction_srv;
  rclcpp::CallbackGroup::SharedPtr _service_group;
};

} // namespace mission_bt_server
