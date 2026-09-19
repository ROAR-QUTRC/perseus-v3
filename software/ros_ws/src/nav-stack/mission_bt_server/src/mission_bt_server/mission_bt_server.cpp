/// @file mission_bt_server.cpp
/// @brief Implementation of MissionBtServer.

#include "mission_bt_server/mission_bt_server/mission_bt_server.hpp"

#include <stdexcept>

namespace mission_bt_server
{

    MissionBtServer::MissionBtServer()
        : Node("mission_bt_server")
    {
        _bt_xml_path = declare_parameter<std::string>("bt_xml_path", "");
        if (_bt_xml_path.empty())
        {
            RCLCPP_FATAL(get_logger(), "bt_xml_path is required");
            throw std::runtime_error("mission_bt_server: bt_xml_path is required");
        }

        _excavation_service_name = declare_parameter<std::string>(
            "excavation_service_name", "/arena/request_excavation_waypoint");
        _construction_service_name = declare_parameter<std::string>(
            "construction_service_name", "/arena/request_construction_waypoint");

        // Matches nav2_behavior_tree::BtActionServer's own defaults (bt_loop_duration
        // 10ms, default_server_timeout/default_cancel_timeout 20s,
        // wait_for_service_timeout 1s) so this tree behaves the same as any other
        // nav2 BT with respect to how fast it ticks and how long it tolerates a slow
        // server ack - only the overall navigation itself is unbounded by these (see
        // the class doc).
        _bt_loop_duration = std::chrono::milliseconds(
            declare_parameter<int>("bt_loop_duration_ms", 10));
        _server_timeout = std::chrono::milliseconds(
            declare_parameter<int>("default_server_timeout_ms", 20000));
        _cancel_timeout = std::chrono::milliseconds(
            declare_parameter<int>("default_cancel_timeout_ms", 20000));
        _wait_for_service_timeout = std::chrono::milliseconds(
            declare_parameter<int>("wait_for_service_timeout_ms", 1000));

        const auto plugin_lib_names = declare_parameter<std::vector<std::string>>(
            "plugin_lib_names",
            std::vector<std::string>{"nav2_navigate_to_pose_action_bt_node",
                                     "request_zone_waypoint_bt_node"});

        rclcpp::NodeOptions bt_node_options;
        bt_node_options.arguments(
            {"--ros-args", "-r", "__node:=mission_bt_server_bt_client", "--"});
        _bt_client_node = std::make_shared<rclcpp::Node>("_", bt_node_options);

        _engine = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(
            plugin_lib_names, _bt_client_node);

        // Reentrant: _run_mission blocks for the whole navigation (seconds to
        // minutes), same reasoning as arena_server's _on_localise - on the default
        // group that would stall the executor thread the other zone's service (or a
        // retry of this one) needs to even be dispatched.
        _service_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        _excavation_srv = create_service<std_srvs::srv::Trigger>(
            "/mission/go_to_excavation_zone",
            std::bind(&MissionBtServer::_on_excavation, this, std::placeholders::_1,
                      std::placeholders::_2),
            rclcpp::ServicesQoS(), _service_group);
        _construction_srv = create_service<std_srvs::srv::Trigger>(
            "/mission/go_to_construction_zone",
            std::bind(&MissionBtServer::_on_construction, this,
                      std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS(), _service_group);

        RCLCPP_INFO(get_logger(),
                    "mission_bt_server up: /mission/go_to_excavation_zone -> %s, "
                    "/mission/go_to_construction_zone -> %s, tree %s",
                    _excavation_service_name.c_str(),
                    _construction_service_name.c_str(), _bt_xml_path.c_str());
    }

    void MissionBtServer::_run_mission(
        const std::string& service_name,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        auto blackboard = BT::Blackboard::create();
        blackboard->set<rclcpp::Node::SharedPtr>("node", _bt_client_node);
        blackboard->set<std::chrono::milliseconds>("server_timeout", _server_timeout);
        blackboard->set<std::chrono::milliseconds>("cancel_timeout", _cancel_timeout);
        blackboard->set<std::chrono::milliseconds>("bt_loop_duration",
                                                   _bt_loop_duration);
        blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout",
                                                   _wait_for_service_timeout);
        // Read by RequestZoneWaypointBtNode's service_name="{service_name}" port -
        // must be set before createTreeFromFile, since BtServiceNode resolves it in
        // its own constructor, at tree-build time.
        blackboard->set<std::string>("service_name", service_name);

        BT::Tree tree;
        try
        {
            tree = _engine->createTreeFromFile(_bt_xml_path, blackboard);
        }
        catch (const std::exception& e)
        {
            response->success = false;
            response->message = std::string("failed to build mission tree: ") + e.what();
            RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
            return;
        }

        auto on_loop = []() {};
        auto is_canceling = []()
        { return !rclcpp::ok(); };
        const auto status =
            _engine->run(&tree, on_loop, is_canceling, _bt_loop_duration);
        _engine->haltAllActions(tree);

        switch (status)
        {
        case nav2_behavior_tree::BtStatus::SUCCEEDED:
            response->success = true;
            response->message = "arrived";
            break;
        case nav2_behavior_tree::BtStatus::FAILED:
            response->success = false;
            response->message =
                "mission tree failed - see this node's log for which step";
            break;
        case nav2_behavior_tree::BtStatus::CANCELED:
            response->success = false;
            response->message = "canceled (node shutting down)";
            break;
        }
        RCLCPP_INFO(get_logger(), "%s -> %s", service_name.c_str(),
                    response->message.c_str());
    }

    void MissionBtServer::_on_excavation(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        _run_mission(_excavation_service_name, response);
    }

    void MissionBtServer::_on_construction(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        _run_mission(_construction_service_name, response);
    }

}  // namespace mission_bt_server
