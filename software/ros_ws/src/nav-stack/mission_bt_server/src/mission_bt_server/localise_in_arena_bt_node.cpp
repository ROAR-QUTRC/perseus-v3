/// @file localise_in_arena_bt_node.cpp
/// @brief Implementation and BT.CPP plugin registration for LocaliseInArenaBtNode.

#include "mission_bt_server/mission_bt_server/localise_in_arena_bt_node.hpp"

#include <behaviortree_cpp/bt_factory.h>

namespace mission_bt_server
{

    LocaliseInArenaBtNode::LocaliseInArenaBtNode(const std::string& service_node_name,
                                                 const BT::NodeConfiguration& conf)
        : nav2_behavior_tree::BtServiceNode<interfaces::srv::LocaliseInArena>(
              service_node_name, conf)
    {
    }

    BT::PortsList LocaliseInArenaBtNode::providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<double>("timeout_s", 1.5,
                                  "how long arena_server waits for a frame with two "
                                  "markers; <= 0 uses its own default"),
        });
    }

    void LocaliseInArenaBtNode::on_tick()
    {
        double timeout_s = 1.5;
        getInput("timeout_s", timeout_s);
        request_->timeout_s = timeout_s;
    }

    BT::NodeStatus LocaliseInArenaBtNode::on_completion(
        std::shared_ptr<interfaces::srv::LocaliseInArena::Response> response)
    {
        if (!response->success)
        {
            // arena_server has already warned why; at one look per scan step this is
            // the expected outcome until the markers come into view.
            RCLCPP_DEBUG(node_->get_logger(), "%s: %s", service_name_.c_str(),
                         response->message.c_str());
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(node_->get_logger(), "%s: %s", service_name_.c_str(),
                    response->message.c_str());
        return BT::NodeStatus::SUCCESS;
    }

}  // namespace mission_bt_server

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<mission_bt_server::LocaliseInArenaBtNode>("LocaliseInArena");
}
