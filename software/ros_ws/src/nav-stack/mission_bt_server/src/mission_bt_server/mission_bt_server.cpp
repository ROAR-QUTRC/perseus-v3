/// @file mission_bt_server.cpp
/// @brief Implementation of MissionBtServer.

#include "mission_bt_server/mission_bt_server/mission_bt_server.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <stdexcept>

namespace mission_bt_server
{

    namespace
    {
        using StartMission = interfaces::srv::StartMission;
        using MissionStatus = interfaces::msg::MissionStatus;

        bool navigation_only(const StartMission::Request& request)
        {
            return request.mode == StartMission::Request::MODE_NAVIGATION_ONLY;
        }

        /// @brief True for the tasks that drive to both zones. task only means
        /// something in NAVIGATION_ONLY, so it is checked only there -- a DUMP_ONLY
        /// request's task is whatever the panel left in it.
        bool is_cycle(const StartMission::Request& request)
        {
            return request.mode == StartMission::Request::MODE_FULL_AUTONOMY ||
                   (navigation_only(request) &&
                    request.task == StartMission::Request::TASK_CYCLE);
        }

        bool needs_excavation(const StartMission::Request& request)
        {
            return is_cycle(request) ||
                   (navigation_only(request) &&
                    request.task == StartMission::Request::TASK_GO_TO_EXCAVATION);
        }

        bool needs_construction(const StartMission::Request& request)
        {
            return is_cycle(request) ||
                   (navigation_only(request) &&
                    request.task == StartMission::Request::TASK_GO_TO_CONSTRUCTION);
        }

        /// @brief A picked point with its stamp zeroed. NavigateToPose resolves the
        /// goal's frame through TF at the goal's stamp, and a point picked minutes
        /// before Start would have aged out of the TF buffer. Zero means "latest".
        geometry_msgs::msg::PoseStamped latest(geometry_msgs::msg::PoseStamped pose)
        {
            pose.header.stamp = builtin_interfaces::msg::Time();
            return pose;
        }

        /// @brief The tree's cycles_done, whatever numeric type the Script node left
        /// it as -- BT.CPP's script arithmetic may store the result as a double.
        uint32_t read_cycles_done(const BT::Blackboard::Ptr& blackboard)
        {
            try
            {
                return static_cast<uint32_t>(std::max(0, blackboard->get<int>("cycles_done")));
            }
            catch (const std::exception&)
            {
            }
            try
            {
                return static_cast<uint32_t>(
                    std::max(0.0, std::round(blackboard->get<double>("cycles_done"))));
            }
            catch (const std::exception&)
            {
                return 0;
            }
        }

        std::string read_phase(const BT::Blackboard::Ptr& blackboard)
        {
            try
            {
                return blackboard->get<std::string>("phase");
            }
            catch (const std::exception&)
            {
                return "";
            }
        }
    }  // namespace

    MissionBtServer::MissionBtServer()
        : Node("mission_bt_server")
    {
        _bt_xml_path = declare_parameter<std::string>("bt_xml_path", "");
        if (_bt_xml_path.empty())
        {
            RCLCPP_FATAL(get_logger(), "bt_xml_path is required");
            throw std::runtime_error("mission_bt_server: bt_xml_path is required");
        }
        _mission_bt_xml_path = declare_parameter<std::string>("mission_bt_xml_path", "");
        if (_mission_bt_xml_path.empty())
        {
            RCLCPP_FATAL(get_logger(), "mission_bt_xml_path is required");
            throw std::runtime_error("mission_bt_server: mission_bt_xml_path is required");
        }

        _excavation_service_name = declare_parameter<std::string>(
            "excavation_service_name", "/arena/request_excavation_waypoint");
        _construction_service_name = declare_parameter<std::string>(
            "construction_service_name", "/arena/request_construction_waypoint");

        // PrepareBucket (StartMission.prepare_bucket): the bucket controller's action,
        // and the pose it leaves the bucket in for driving. The default pose keeps the
        // bucket out of the MID-360's and D455's view of the ground from 2 m ahead
        // while leaving ~0.17 m of ground clearance; see mission.xml.
        _bucket_action_name = declare_parameter<std::string>(
            "bucket_action_name", "/payloads/bucket_trajectory_controller/follow_joint_trajectory");
        _bucket_travel_lift_deg = declare_parameter<double>("bucket_travel_lift_deg", 25.0);
        _bucket_travel_tilt_deg = declare_parameter<double>("bucket_travel_tilt_deg", -20.0);
        _bucket_travel_jaw_deg = declare_parameter<double>("bucket_travel_jaw_deg", 0.0);
        // Per move. Generous: the controller has to get there within this plus its
        // 3 s goal_time tolerance, from wherever the bucket was left.
        _bucket_move_s = declare_parameter<double>("bucket_move_s", 10.0);

        // DigBucket (StartMission MODE_DIG_ONLY); the sequence is in mission.xml. Joint
        // angles in degrees as for the travel pose: tilt positive tips the bucket down,
        // lift positive lowers the arms.
        //   dig_tilt_deg        cutting edge down, before the arms lower. The tilt joint
        //                       stops at 28.1 deg (bucket.urdf.xacro's tilt_upper), and a
        //                       target it cannot reach fails the move on the controller's
        //                       2 deg goal tolerance, so 28 is as far as it goes.
        //   dig_lift_deg        arms down into the regolith: at tilt 28 the tray floor
        //                       meets grade near lift 30 and sits ~5 cm under at 35.
        //   dig_push_m          creep straight ahead with the edge in, at dig_speed.
        //   dig_curl_tilt_deg   curl the bucket up to hold the load.
        //   dig_carry_m         creep on while the arms rise to dig_carry_lift_deg, the
        //                       carrying pose: 0 is arms level at the top, so the curled
        //                       bucket rides as high as it goes. The rise takes the
        //                       longer of the drive and bucket_move_s, so a long lift is
        //                       not rushed to fit a short creep.
        //   dig_speed           both creeps, m/s. The ESCs stall below ~0.34 rad/s at the
        //                       wheel (~0.05 m/s), so keep it clear of that.
        _dig_tilt_deg = declare_parameter<double>("dig_tilt_deg", 28.0);
        _dig_lift_deg = declare_parameter<double>("dig_lift_deg", 32.0);
        _dig_push_m = declare_parameter<double>("dig_push_m", 0.20);
        _dig_curl_tilt_deg = declare_parameter<double>("dig_curl_tilt_deg", -28.0);
        _dig_carry_m = declare_parameter<double>("dig_carry_m", 0.40);
        _dig_carry_lift_deg = declare_parameter<double>("dig_carry_lift_deg", 0.0);
        _dig_speed = declare_parameter<double>("dig_speed", 0.08);

        // Matches nav2_behavior_tree::BtActionServer's own defaults (bt_loop_duration
        // 10ms, default_server_timeout/default_cancel_timeout 20s,
        // wait_for_service_timeout 1s) so this tree behaves the same as any other
        // nav2 BT with respect to how fast it ticks and how long it tolerates a slow
        // server ack - only the overall navigation itself is unbounded by these.
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
                                     "nav2_drive_on_heading_bt_node",
                                     "request_zone_waypoint_bt_node", "move_bucket_bt_node"});

        rclcpp::NodeOptions bt_node_options;
        bt_node_options.arguments(
            {"--ros-args", "-r", "__node:=mission_bt_server_bt_client", "--"});
        _bt_client_node = std::make_shared<rclcpp::Node>("_", bt_node_options);

        _engine = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(
            plugin_lib_names, _bt_client_node);

        // Latched: a panel opened mid-mission (or reconnecting) sees where things
        // stand at once instead of waiting for the next phase change.
        _status_pub = create_publisher<MissionStatus>(
            "/mission/status", rclcpp::QoS(1).reliable().transient_local());
        _last_status.state = MissionStatus::STATE_IDLE;
        _publish_status(_last_status, true);

        // Reentrant: the single-trip services block for a whole navigation, and on
        // the default group that would stall the executor thread /mission/stop needs
        // to be dispatched on.
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
        _start_srv = create_service<StartMission>(
            "/mission/start",
            std::bind(&MissionBtServer::_on_start, this, std::placeholders::_1,
                      std::placeholders::_2),
            rclcpp::ServicesQoS(), _service_group);
        _stop_srv = create_service<std_srvs::srv::Trigger>(
            "/mission/stop",
            std::bind(&MissionBtServer::_on_stop, this, std::placeholders::_1,
                      std::placeholders::_2),
            rclcpp::ServicesQoS(), _service_group);

        RCLCPP_INFO(get_logger(),
                    "mission_bt_server up: /mission/start (%s), /mission/stop, "
                    "/mission/go_to_excavation_zone -> %s, "
                    "/mission/go_to_construction_zone -> %s (%s)",
                    _mission_bt_xml_path.c_str(), _excavation_service_name.c_str(),
                    _construction_service_name.c_str(), _bt_xml_path.c_str());
    }

    MissionBtServer::~MissionBtServer()
    {
        _stop_requested = true;
        if (_worker.joinable())
        {
            _worker.join();
        }
    }

    BT::Blackboard::Ptr MissionBtServer::_make_blackboard() const
    {
        auto blackboard = BT::Blackboard::create();
        blackboard->set<rclcpp::Node::SharedPtr>("node", _bt_client_node);
        blackboard->set<std::chrono::milliseconds>("server_timeout", _server_timeout);
        blackboard->set<std::chrono::milliseconds>("cancel_timeout", _cancel_timeout);
        blackboard->set<std::chrono::milliseconds>("bt_loop_duration",
                                                   _bt_loop_duration);
        blackboard->set<std::chrono::milliseconds>("wait_for_service_timeout",
                                                   _wait_for_service_timeout);
        return blackboard;
    }

    nav2_behavior_tree::BtStatus
    MissionBtServer::_run_tree(BT::Tree& tree, const std::function<void()>& on_loop)
    {
        auto is_canceling = [this]()
        { return _stop_requested.load() || !rclcpp::ok(); };
        const auto status = _engine->run(&tree, on_loop, is_canceling, _bt_loop_duration);
        // Cancels whatever action is still in flight -- on a stop, the
        // NavigateToPose goal the rover is driving on.
        _engine->haltAllActions(tree);
        return status;
    }

    // ---------------------------------------------------------------------------
    // The original single-trip services
    // ---------------------------------------------------------------------------

    void MissionBtServer::_run_zone_trip(
        const std::string& service_name,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (_busy.exchange(true))
        {
            response->success = false;
            response->message = "a mission is already running - stop it first";
            return;
        }
        _stop_requested = false;

        auto blackboard = _make_blackboard();
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
            response->message = std::string("failed to build zone tree: ") + e.what();
            RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
            _busy = false;
            return;
        }

        switch (_run_tree(tree, []() {}))
        {
        case nav2_behavior_tree::BtStatus::SUCCEEDED:
            response->success = true;
            response->message = "arrived";
            break;
        case nav2_behavior_tree::BtStatus::FAILED:
            response->success = false;
            response->message = "zone tree failed - see this node's log for which step";
            break;
        case nav2_behavior_tree::BtStatus::CANCELED:
            response->success = false;
            response->message = "stopped";
            break;
        }
        RCLCPP_INFO(get_logger(), "%s -> %s", service_name.c_str(),
                    response->message.c_str());
        _busy = false;
    }

    void MissionBtServer::_on_excavation(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        _run_zone_trip(_excavation_service_name, response);
    }

    void MissionBtServer::_on_construction(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        _run_zone_trip(_construction_service_name, response);
    }

    // ---------------------------------------------------------------------------
    // The mission
    // ---------------------------------------------------------------------------

    std::string MissionBtServer::_validate(const StartMission::Request& request)
    {
        if (request.mode != StartMission::Request::MODE_FULL_AUTONOMY &&
            request.mode != StartMission::Request::MODE_NAVIGATION_ONLY &&
            request.mode != StartMission::Request::MODE_DUMP_ONLY &&
            request.mode != StartMission::Request::MODE_DIG_ONLY)
        {
            return "unknown mode " + std::to_string(request.mode);
        }
        if (request.mode == StartMission::Request::MODE_NAVIGATION_ONLY &&
            request.task != StartMission::Request::TASK_CYCLE &&
            request.task != StartMission::Request::TASK_GO_TO_EXCAVATION &&
            request.task != StartMission::Request::TASK_GO_TO_CONSTRUCTION)
        {
            return "unknown task " + std::to_string(request.task);
        }
        if (is_cycle(request) && request.cycles < 1)
        {
            return "cycles must be at least 1";
        }
        if (needs_excavation(request) && !request.use_arena_excavation &&
            request.excavation_point.header.frame_id.empty())
        {
            return "no excavation point - pick one on the map, or use arena_server's";
        }
        if (needs_construction(request) && !request.use_arena_construction &&
            request.construction_point.header.frame_id.empty())
        {
            return "no construction point - pick one on the map, or use arena_server's";
        }
        return "";
    }

    std::string MissionBtServer::_task_name(const StartMission::Request& request)
    {
        if (request.mode == StartMission::Request::MODE_FULL_AUTONOMY)
        {
            return "full_cycle";
        }
        if (request.mode == StartMission::Request::MODE_DUMP_ONLY)
        {
            return "dump";
        }
        if (request.mode == StartMission::Request::MODE_DIG_ONLY)
        {
            return "dig";
        }
        switch (request.task)
        {
        case StartMission::Request::TASK_GO_TO_EXCAVATION:
            return "nav_excavation";
        case StartMission::Request::TASK_GO_TO_CONSTRUCTION:
            return "nav_construction";
        default:
            return "nav_cycle";
        }
    }

    void MissionBtServer::_on_start(const std::shared_ptr<StartMission::Request> request,
                                    std::shared_ptr<StartMission::Response> response)
    {
        const std::string problem = _validate(*request);
        if (!problem.empty())
        {
            response->accepted = false;
            response->message = problem;
            RCLCPP_WARN(get_logger(), "refused mission: %s", problem.c_str());
            return;
        }
        if (_busy.exchange(true))
        {
            response->accepted = false;
            response->message = "a mission is already running - stop it first";
            return;
        }

        // The previous worker has already released _busy, so it is finished or at
        // its last statement: this join is immediate, never a wait on a mission.
        if (_worker.joinable())
        {
            _worker.join();
        }
        _stop_requested = false;
        _worker = std::thread(&MissionBtServer::_run_mission, this, *request);

        response->accepted = true;
        response->message = "mission started";
    }

    void MissionBtServer::_on_stop(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!_busy.load())
        {
            response->success = false;
            response->message = "nothing is running";
            return;
        }
        _stop_requested = true;
        response->success = true;
        response->message = "stopping";
        RCLCPP_INFO(get_logger(), "stop requested");
    }

    void MissionBtServer::_run_mission(StartMission::Request request)
    {
        const bool cycling = is_cycle(request);
        MissionStatus status;
        status.state = MissionStatus::STATE_RUNNING;
        status.mode = request.mode;
        status.task = request.task;
        status.cycles_target = cycling ? request.cycles : 0;
        status.cycles_completed = 0;
        status.phase = "starting";
        _publish_status(status, true);

        auto blackboard = _make_blackboard();
        blackboard->set<std::string>("task", _task_name(request));
        blackboard->set<int>("cycles", static_cast<int>(std::max<uint32_t>(1, request.cycles)));
        blackboard->set<bool>("use_arena_excavation", request.use_arena_excavation);
        blackboard->set<bool>("use_arena_construction", request.use_arena_construction);
        blackboard->set<geometry_msgs::msg::PoseStamped>("excavation_point",
                                                         latest(request.excavation_point));
        blackboard->set<geometry_msgs::msg::PoseStamped>(
            "construction_point", latest(request.construction_point));
        // Both RequestZoneWaypoint nodes resolve service_name when the tree is
        // built, so both must be set even when that zone uses a picked point.
        blackboard->set<std::string>("excavation_service", _excavation_service_name);
        blackboard->set<std::string>("construction_service", _construction_service_name);
        blackboard->set<int>("cycles_done", 0);
        blackboard->set<std::string>("phase", "starting");
        blackboard->set<bool>("prepare_bucket", request.prepare_bucket);
        blackboard->set<std::string>("bucket_action", _bucket_action_name);
        blackboard->set<double>("bucket_travel_lift_deg", _bucket_travel_lift_deg);
        blackboard->set<double>("bucket_travel_tilt_deg", _bucket_travel_tilt_deg);
        blackboard->set<double>("bucket_travel_jaw_deg", _bucket_travel_jaw_deg);
        blackboard->set<double>("bucket_move_s", _bucket_move_s);
        blackboard->set<double>("dig_tilt_deg", _dig_tilt_deg);
        blackboard->set<double>("dig_lift_deg", _dig_lift_deg);
        blackboard->set<double>("dig_push_m", _dig_push_m);
        blackboard->set<double>("dig_curl_tilt_deg", _dig_curl_tilt_deg);
        blackboard->set<double>("dig_carry_m", _dig_carry_m);
        blackboard->set<double>("dig_carry_lift_deg", _dig_carry_lift_deg);
        blackboard->set<double>("dig_speed", _dig_speed);
        // The carry's drive and lift start together; the lift gets at least
        // bucket_move_s, since 32 -> 0 deg in a 5 s creep would outrun the rams.
        const double carry_s = _dig_carry_m / std::max(_dig_speed, 0.01);
        blackboard->set<double>("dig_carry_s", std::max(carry_s, _bucket_move_s));
        // DriveOnHeading gives up after time_allowance; leave room for the ramp-up.
        blackboard->set<double>("dig_push_allowance_s",
                                _dig_push_m / std::max(_dig_speed, 0.01) + 10.0);
        blackboard->set<double>("dig_carry_allowance_s", carry_s + 10.0);

        RCLCPP_INFO(get_logger(),
                    "mission: task %s, %u cycle(s), excavation %s, construction %s, "
                    "bucket %s",
                    _task_name(request).c_str(), request.cycles,
                    request.use_arena_excavation ? "arena" : "picked",
                    request.use_arena_construction ? "arena" : "picked",
                    request.prepare_bucket ? "to travel pose first" : "left as is");

        BT::Tree tree;
        try
        {
            tree = _engine->createTreeFromFile(_mission_bt_xml_path, blackboard);
        }
        catch (const std::exception& e)
        {
            status.state = MissionStatus::STATE_FAILED;
            status.message = std::string("failed to build mission tree: ") + e.what();
            RCLCPP_ERROR(get_logger(), "%s", status.message.c_str());
            _publish_status(status, true);
            _busy = false;
            return;
        }

        // Called every tick: republishes only when the phase or the count moved.
        auto on_loop = [&]()
        {
            status.phase = read_phase(blackboard);
            status.cycles_completed = read_cycles_done(blackboard);
            _publish_status(status);
        };
        const auto result = _run_tree(tree, on_loop);

        status.cycles_completed = read_cycles_done(blackboard);
        const std::string last_phase = read_phase(blackboard);
        switch (result)
        {
        case nav2_behavior_tree::BtStatus::SUCCEEDED:
            status.state = MissionStatus::STATE_SUCCEEDED;
            status.phase = "done";
            status.message = cycling ? "completed " + std::to_string(status.cycles_completed) +
                                           " of " + std::to_string(status.cycles_target) +
                                           " cycle(s)"
                                     : "arrived";
            break;
        case nav2_behavior_tree::BtStatus::FAILED:
            status.state = MissionStatus::STATE_FAILED;
            status.message = "failed during " + last_phase +
                             " - see mission_bt_server's log for which step";
            break;
        case nav2_behavior_tree::BtStatus::CANCELED:
            status.state = MissionStatus::STATE_STOPPED;
            status.message = "stopped during " + last_phase;
            break;
        }
        RCLCPP_INFO(get_logger(), "mission -> %s", status.message.c_str());
        _publish_status(status, true);
        _busy = false;
    }

    void MissionBtServer::_publish_status(MissionStatus status, bool force)
    {
        std::lock_guard<std::mutex> lock(_status_mutex);
        const bool changed = status.state != _last_status.state ||
                             status.phase != _last_status.phase ||
                             status.cycles_completed != _last_status.cycles_completed ||
                             status.message != _last_status.message;
        if (!changed && !force)
        {
            return;
        }
        status.header.stamp = now();
        _last_status = status;
        _status_pub->publish(status);
    }

}  // namespace mission_bt_server
