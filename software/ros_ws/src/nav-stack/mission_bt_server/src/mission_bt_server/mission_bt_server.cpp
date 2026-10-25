/// @file mission_bt_server.cpp
/// @brief Implementation of MissionBtServer.

#include "mission_bt_server/mission_bt_server/mission_bt_server.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <stdexcept>

#include "mission_bt_server/mission_bt_server/bucket_trees.hpp"

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

        /// @brief Full autonomy's drive cycle with a timed stop at each zone in place
        /// of the bucket, for a rover with no bucket feedback.
        bool timed_autonomy(const StartMission::Request& request)
        {
            return request.mode == StartMission::Request::MODE_FULL_AUTONOMY_TIMED;
        }

        /// @brief True for the tasks that drive to both zones. task only means
        /// something in NAVIGATION_ONLY, so it is checked only there -- a DUMP_ONLY
        /// request's task is whatever the panel left in it.
        bool is_cycle(const StartMission::Request& request)
        {
            return request.mode == StartMission::Request::MODE_FULL_AUTONOMY ||
                   timed_autonomy(request) ||
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
        _bucket_bt_dir = declare_parameter<std::string>(
            "bucket_bt_dir",
            (std::filesystem::path(_mission_bt_xml_path).parent_path() / "bucket").string());

        _excavation_service_name = declare_parameter<std::string>(
            "excavation_service_name", "/arena/request_excavation_waypoint");
        _construction_service_name = declare_parameter<std::string>(
            "construction_service_name", "/arena/request_construction_waypoint");

        // PrepareBucket's travel pose, DigBucket's pose and creeps, and the nav2 BT
        // timeouts: shared with bucket_cli, which runs the same subtrees.
        _bucket = BucketMissionParams::declare(*this);
        _timeouts = BtTimeouts::declare(*this);
        _default_zone_pause_s = declare_parameter<double>("default_zone_pause_s", 2.0);

        // ScanAndLocalise: turn until two rail markers are in view, for at most
        // scan_max_deg (a full turn by default) per attempt and scan_retries attempts.
        _scan_max_deg = declare_parameter<double>("scan_max_deg", 360.0);
        _scan_retries = declare_parameter<int>("scan_retries", 2);
        _scan_look_s = declare_parameter<double>("scan_look_s", 1.5);
        _scan_wall_range_m = declare_parameter<double>("scan_wall_range_m", 4.0);
        // The rover turns about base_footprint and its corner sweeps 0.93 m from it, so a
        // wall nearer than that is hit. The margin is the rest. 0 or less turns without
        // the check.
        _scan_min_clearance_m = declare_parameter<double>("scan_min_clearance_m", 1.0);
        if (_scan_max_deg <= 0.0 || _scan_retries < 1)
        {
            RCLCPP_FATAL(get_logger(), "scan_max_deg must be positive and scan_retries >= 1");
            throw std::runtime_error("mission_bt_server: bad scan_max_deg / scan_retries");
        }

        // Same names as bt_navigator's, whose own monitoring is off by default, so the
        // default port is free for this node.
        _groot_monitoring = declare_parameter<bool>("enable_groot_monitoring", true);
        const auto groot_port = declare_parameter<int>("groot_server_port", 1667);
        if (groot_port < 1 || groot_port > 65534)
        {
            RCLCPP_FATAL(get_logger(), "groot_server_port must be in 1-65534, got %ld",
                         groot_port);
            throw std::runtime_error("mission_bt_server: groot_server_port out of range");
        }
        _groot_server_port = static_cast<uint16_t>(groot_port);

        const auto plugin_lib_names = declare_parameter<std::vector<std::string>>(
            "plugin_lib_names",
            std::vector<std::string>{"nav2_navigate_to_pose_action_bt_node",
                                     "nav2_drive_on_heading_bt_node",
                                     "nav2_wait_action_bt_node",
                                     "nav2_clear_costmap_service_bt_node",
                                     "request_zone_waypoint_bt_node", "move_bucket_bt_node",
                                     "blind_jaw_bt_node", "localise_in_arena_bt_node",
                                     "wall_side_bt_node", "wait_for_markers_bt_node",
                                     "blind_spin_bt_node"});

        rclcpp::NodeOptions bt_node_options;
        bt_node_options.arguments(
            {"--ros-args", "-r", "__node:=mission_bt_server_bt_client", "--"});
        _bt_client_node = std::make_shared<rclcpp::Node>("_", bt_node_options);

        _engine = std::make_unique<MissionBtEngine>(
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

        if (_groot_monitoring)
        {
            RCLCPP_INFO(get_logger(), "Groot2 monitoring on port %u while a tree runs",
                        _groot_server_port);
        }
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
        _timeouts.apply(*blackboard, _bt_client_node);
        return blackboard;
    }

    nav2_behavior_tree::BtStatus
    MissionBtServer::_run_tree(BT::Tree& tree, const std::function<void()>& on_loop)
    {
        if (_groot_monitoring)
        {
            // A port that will not bind costs the live view, never the mission.
            try
            {
                _engine->addGrootMonitoring(&tree, _groot_server_port);
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(get_logger(), "Groot2 monitoring unavailable on port %u: %s",
                            _groot_server_port, e.what());
            }
        }

        auto is_canceling = [this]()
        { return _stop_requested.load() || !rclcpp::ok(); };
        const auto status = _engine->run(&tree, on_loop, is_canceling, _timeouts.bt_loop_duration);
        // Cancels whatever action is still in flight -- on a stop, the
        // NavigateToPose goal the rover is driving on.
        _engine->haltAllActions(tree);
        // The publisher holds a reference to the tree, which the caller is about to
        // destroy; this also frees the port for the next run.
        _engine->resetGrootMonitor();
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
            request.mode != StartMission::Request::MODE_DIG_ONLY &&
            !timed_autonomy(request))
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
        if (timed_autonomy(request))
        {
            return "full_cycle_timed";
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
        // The timed mode exists for a rover with no bucket feedback, so it never
        // moves the bucket - not even the travel pose a client may have left ticked.
        const bool prepare_bucket = request.prepare_bucket && !timed_autonomy(request);
        blackboard->set<bool>("prepare_bucket", prepare_bucket);
        const double pause_s =
            request.zone_pause_s > 0.0 ? request.zone_pause_s : _default_zone_pause_s;
        blackboard->set<unsigned int>("pause_ms",
                                      static_cast<unsigned int>(std::lround(pause_s * 1000.0)));
        _bucket.apply(*blackboard);
        // Dump and dig run where the rover stands and need no arena frame.
        const bool scan_localise = request.scan_localise &&
                                   request.mode != StartMission::Request::MODE_DUMP_ONLY &&
                                   request.mode != StartMission::Request::MODE_DIG_ONLY;
        blackboard->set<bool>("scan_localise", scan_localise);
        blackboard->set<double>("scan_spin_deg", _scan_max_deg);
        blackboard->set<double>("scan_wall_range_m", _scan_wall_range_m);
        blackboard->set<double>("scan_min_clearance_m", _scan_min_clearance_m);
        blackboard->set<double>("scan_look_s", _scan_look_s);
        blackboard->set<int>("scan_retries", _scan_retries);
        // BlindSpin turns at 0.4 rad/s; allow for half that speed, plus slack.
        blackboard->set<double>("scan_spin_allowance_s",
                                _scan_max_deg * M_PI / 180.0 / 0.2 + 5.0);

        RCLCPP_INFO(get_logger(),
                    "mission: task %s, %u cycle(s), excavation %s, construction %s, "
                    "bucket %s, %s",
                    _task_name(request).c_str(), request.cycles,
                    request.use_arena_excavation ? "arena" : "picked",
                    request.use_arena_construction ? "arena" : "picked",
                    prepare_bucket ? "to travel pose first" : "left as is",
                    scan_localise ? "scan for markers first" : "no marker scan");
        if (timed_autonomy(request))
        {
            RCLCPP_INFO(get_logger(), "mission: no bucket moves, %.1f s stop at each zone",
                        pause_s);
        }

        // Registered afresh every start, so edits to mission.xml or the bucket trees
        // take effect on the next mission, as a single createTreeFromFile would. The
        // registrations are cleared again either way: the zone trips build theirs
        // with createTreeFromFile, which warns when any are left behind.
        BT::Tree tree;
        auto& factory = _engine->factory();
        try
        {
            factory.clearRegisteredBehaviorTrees();
            register_bucket_trees(factory, _bucket_bt_dir);
            register_tree_file(factory, _mission_bt_xml_path);
            // mission.xml's (and mission_lift_only.xml's) main_tree_to_execute.
            tree = factory.createTree("MainTree", blackboard);
            factory.clearRegisteredBehaviorTrees();
        }
        catch (const std::exception& e)
        {
            factory.clearRegisteredBehaviorTrees();
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
