#pragma once

/// @file mission_bt_server.hpp
/// @brief Hosts the mission behaviour trees behind services for the RViz mission panel.

#include <atomic>
#include <chrono>
#include <interfaces/msg/mission_status.hpp>
#include <interfaces/srv/start_mission.hpp>
#include <memory>
#include <mutex>
#include <nav2_behavior_tree/behavior_tree_engine.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>
#include <vector>

namespace mission_bt_server
{

    /// @brief Runs the mission behaviour trees for the RViz Mission Control panel.
    ///
    /// Two ways in, sharing one engine and one "busy" flag, so at most one tree ever
    /// runs at a time whichever way it was started:
    ///
    ///   /mission/start (interfaces/StartMission) runs mission.xml -- full autonomy
    ///   or navigation only, cycling between the zones or a single trip, to points
    ///   picked in RViz or ones arena_server chooses. It returns as soon as the
    ///   mission is accepted; the tree runs on a worker thread, reports progress on
    ///   /mission/status (latched) and is cancelled by /mission/stop.
    ///
    ///   /mission/go_to_excavation_zone and /mission/go_to_construction_zone
    ///   (std_srvs/Trigger) are the original two buttons' services: one blocking
    ///   call per trip, running go_to_zone_waypoint.xml (arena_server's point, then
    ///   NavigateToPose). Kept unchanged for anything still calling them; they are
    ///   refused while a mission runs, and /mission/stop cancels them too.
    ///
    /// Deliberately not a nav2_behavior_tree::BtActionServer<...>: that class is
    /// built around a LifecycleNode and its own dedicated ROS action, which is
    /// more machinery than a start/stop panel needs. This node instead replicates
    /// just the blackboard setup nav2_behavior_tree's own BT nodes require (see
    /// BtActionServer::on_configure in bt_action_server_impl.hpp) and calls
    /// BehaviorTreeEngine::run() directly.
    ///
    /// Stopping works through run()'s cancel callback: once /mission/stop sets the
    /// flag, the engine returns CANCELED at the next tick and haltAllActions()
    /// halts the tree, which cancels the in-flight NavigateToPose goal at
    /// bt_navigator -- so the rover stops, rather than finishing the leg it was on.
    class MissionBtServer : public rclcpp::Node
    {
    public:
        MissionBtServer();

        /// @brief Stops a running mission and joins its worker thread.
        ~MissionBtServer() override;

    private:
        /// @brief Blackboard entries every nav2_behavior_tree BT node reads in its
        /// own constructor, on a fresh blackboard.
        BT::Blackboard::Ptr _make_blackboard() const;

        /// @brief Runs @p tree to completion or cancellation. @p on_loop is called
        /// every tick.
        nav2_behavior_tree::BtStatus _run_tree(BT::Tree& tree,
                                               const std::function<void()>& on_loop);

        // ---- the original single-trip services ----

        /// @brief Builds go_to_zone_waypoint.xml with service_name pointed at one
        /// zone and runs it to completion, blocking the call.
        void _run_zone_trip(const std::string& service_name,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        void
        _on_excavation(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);
        void _on_construction(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        // ---- the mission ----

        /// @brief Validates the request, and if it is runnable claims the busy flag
        /// and starts _run_mission on the worker thread.
        void _on_start(const std::shared_ptr<interfaces::srv::StartMission::Request> request,
                       std::shared_ptr<interfaces::srv::StartMission::Response> response);
        void _on_stop(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        /// @brief Worker-thread body: builds mission.xml from @p request, runs it,
        /// and publishes the outcome. Releases the busy flag when done.
        void _run_mission(interfaces::srv::StartMission::Request request);

        /// @brief Why @p request cannot run, or empty if it can.
        static std::string _validate(const interfaces::srv::StartMission::Request& request);

        /// @brief mission.xml's `task` blackboard value for @p request.
        static std::string _task_name(const interfaces::srv::StartMission::Request& request);

        /// @brief Publishes @p status if it differs from the last one published (or
        /// @p force), stamping it first.
        void _publish_status(interfaces::msg::MissionStatus status, bool force = false);

        std::string _bt_xml_path;
        std::string _mission_bt_xml_path;
        std::string _excavation_service_name;
        std::string _construction_service_name;

        // PrepareBucket's action and travel pose; see the constructor.
        std::string _bucket_action_name;
        double _bucket_travel_lift_deg;
        double _bucket_travel_tilt_deg;
        double _bucket_travel_jaw_deg;
        double _bucket_move_s;

        // DigBucket's pose, creeps and carry; see the constructor.
        double _dig_tilt_deg;
        double _dig_lift_deg;
        double _dig_push_m;
        double _dig_curl_tilt_deg;
        double _dig_carry_m;
        double _dig_carry_lift_deg;
        double _dig_speed;

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

        // True while any tree runs, from either entry point. The engine and its
        // client node are not safe to drive from two threads at once, and two
        // NavigateToPose goals would only preempt each other at bt_navigator anyway.
        std::atomic<bool> _busy{false};
        // Set by /mission/stop, read by the engine's cancel callback every tick.
        std::atomic<bool> _stop_requested{false};
        std::thread _worker;

        rclcpp::Publisher<interfaces::msg::MissionStatus>::SharedPtr _status_pub;
        std::mutex _status_mutex;
        interfaces::msg::MissionStatus _last_status;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _excavation_srv;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _construction_srv;
        rclcpp::Service<interfaces::srv::StartMission>::SharedPtr _start_srv;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _stop_srv;
        rclcpp::CallbackGroup::SharedPtr _service_group;
    };

}  // namespace mission_bt_server
