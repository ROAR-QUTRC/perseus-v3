#pragma once

/// @file request_zone_waypoint_bt_node.hpp
/// @brief BT.CPP leaf calling arena_server's RequestZoneWaypoint service.

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interfaces/srv/request_zone_waypoint.hpp>
#include <memory>
#include <nav2_behavior_tree/bt_service_node.hpp>
#include <string>

namespace mission_bt_server
{

    /// @brief Wraps one call to arena_server's /arena/request_*_waypoint services,
    /// handing the returned point to the rest of the tree as a PoseStamped output
    /// port for NavigateToPose to consume.
    ///
    /// One node type serves both zones: which zone it asks is the service_name
    /// port, not something baked in here - mirroring how arena_server itself
    /// exposes one RequestZoneWaypoint.srv under two service names rather than
    /// two message types.
    class RequestZoneWaypointBtNode
        : public nav2_behavior_tree::BtServiceNode<
              interfaces::srv::RequestZoneWaypoint>
    {
    public:
        RequestZoneWaypointBtNode(const std::string& service_node_name,
                                  const BT::NodeConfiguration& conf);

        static BT::PortsList providedPorts();

        /// @brief Copies the service's waypoint onto the "goal" output port on
        /// success; logs and fails the node otherwise (e.g. no costmap yet, zone
        /// fully obstructed - see arena_server's RequestZoneWaypoint.srv).
        BT::NodeStatus on_completion(
            std::shared_ptr<interfaces::srv::RequestZoneWaypoint::Response> response)
            override;
    };

}  // namespace mission_bt_server
