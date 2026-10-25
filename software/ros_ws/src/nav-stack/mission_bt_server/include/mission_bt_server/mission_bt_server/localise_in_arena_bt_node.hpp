#pragma once

/// @file localise_in_arena_bt_node.hpp
/// @brief BT.CPP leaf calling arena_server's LocaliseInArena service.

#include <interfaces/srv/localise_in_arena.hpp>
#include <memory>
#include <nav2_behavior_tree/bt_service_node.hpp>
#include <string>

namespace mission_bt_server
{

    /// @brief One look for the rail fiducials: asks arena_server for a pose fix and
    /// succeeds when it got one, which also re-seeds map -> odom.
    ///
    /// The service is passive. It reports what the camera sees right now, waiting up to
    /// timeout_s for a frame with at least two surveyed markers, and fails rather than
    /// move the rover to find them. Looking around is the tree's job (mission.xml's
    /// ScanAndLocalise), which is why a failure here is an ordinary "not yet".
    class LocaliseInArenaBtNode
        : public nav2_behavior_tree::BtServiceNode<interfaces::srv::LocaliseInArena>
    {
    public:
        LocaliseInArenaBtNode(const std::string& service_node_name,
                              const BT::NodeConfiguration& conf);

        static BT::PortsList providedPorts();

        /// @brief Copies timeout_s into the request.
        void on_tick() override;

        /// @brief SUCCESS with the fix's markers on the log, FAILURE when arena_server
        /// saw fewer than two markers or could not fit them.
        BT::NodeStatus on_completion(
            std::shared_ptr<interfaces::srv::LocaliseInArena::Response> response) override;
    };

}  // namespace mission_bt_server
