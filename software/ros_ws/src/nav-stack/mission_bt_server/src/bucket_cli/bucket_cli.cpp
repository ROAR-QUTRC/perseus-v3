/// @file bucket_cli.cpp
/// @brief Runs the mission's bucket behaviours, and your own, from the command line.
///
/// The same subtrees, plugins and parameter values mission_bt_server runs
/// (mission.xml's PrepareBucket, DigBucket and DumpBucket, and single MoveBucket
/// steps), without the rest of the mission stack - so the bucket can be exercised
/// on the bench, on the rover, or against payloads' rover_can_sim.py on vcan.
///
/// It also runs the bucket behaviours built in Groot2 from saved poses
/// (autonomy_bringup's behavior_trees/bucket/, written by payloads' bucket_teach.py),
/// and every tree it runs can be watched live from Groot2's Monitor.
///
///   ros2 run mission_bt_server bucket_cli dump
///   ros2 run mission_bt_server bucket_cli move --lift 30 --tilt -10
///   ros2 run mission_bt_server bucket_cli dig --drive stub
///   ros2 run mission_bt_server bucket_cli run ExampleDump --rearm
///   ros2 run mission_bt_server bucket_cli pose travel
///   ros2 run mission_bt_server bucket_cli project
///   ros2 run mission_bt_server bucket_cli rearm
///
/// Mission values are ROS parameters, as on mission_bt_server:
///   ros2 run mission_bt_server bucket_cli dig --ros-args -p dig_lift_deg:=30.0

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_cpp/utils/shared_library.h>
#include <behaviortree_cpp/xml_parsing.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <nav2_msgs/action/drive_on_heading.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <set>
#include <sstream>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>
#include <vector>

#include "mission_bt_server/mission_bt_server/bucket_mission_params.hpp"

namespace
{
    using namespace std::chrono_literals;
    using mission_bt_server::BtTimeouts;
    using mission_bt_server::BucketMissionParams;

    const char* const USAGE = R"(usage: bucket_cli COMMAND [options] [--ros-args ...]

commands:
  prepare    mission.xml's PrepareBucket: every joint to 0, then the travel pose
  dig        mission.xml's DigBucket, from where the rover stands (drives ~0.6 m)
  dump       mission.xml's DumpBucket
  zero       one move: lift, tilt and jaw to 0
  travel     one move: the travel pose
  move       one move to --lift/--tilt/--jaw (any of them; the rest hold)
  run TREE   a tree from the bucket behaviours directory (or mission.xml)
  pose NAME  one saved pose: run Pose_NAME
  project    write the directory's Groot2 project, bucket.btproj
  rearm      hand the bucket back from the operator (/bucket/rearm)

options:
  --lift DEG, --tilt DEG, --jaw DEG   move's targets
  --timeout S        how long each move may take to arrive (default: bucket_move_s, 10)
  --attempts N       how many times a move that misses is sent
                     (default: bucket_move_attempts, 2)
  --duration S       move only: a timed move over S seconds instead of a direct one
  --drive MODE       dig's creeps: nav2 (behavior_server's drive_on_heading),
                     stub (pretend: wait out the drive's time, no motion), or
                     auto (nav2 if it answers within 1 s, else stub) [auto]
  --server NAME      the bucket controller's FollowJointTrajectory action
  --xml PATH         mission tree (default: autonomy_bringup's mission.xml)
  --dir PATH         bucket behaviours directory (default: the source copy of
                     autonomy_bringup's behavior_trees/bucket)
  --rearm            hand the bucket back from the operator before running, if
                     the operator has it
  --groot-port P     Groot2 monitoring port while a tree runs; 0 turns it off [1667]
  --start-delay S    wait S seconds before the first tick, to connect Groot2's Monitor
  --hold S           keep the finished tree on Groot2 for S seconds (Ctrl-C ends it)
)";

    std::atomic<bool> g_interrupted{false};

    /// Sleeps for @p seconds, or until Ctrl-C. Returns false if interrupted.
    bool wait_unless_interrupted(double seconds)
    {
        const auto until = std::chrono::steady_clock::now() +
                           std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                               std::chrono::duration<double>(seconds));
        while (std::chrono::steady_clock::now() < until && !g_interrupted && rclcpp::ok())
            std::this_thread::sleep_for(50ms);
        return !g_interrupted;
    }

    /// Stands in for nav2's DriveOnHeading when behavior_server isn't running: waits
    /// as long as the drive would take, so the bucket half of a dig keeps its timing.
    class StubDriveOnHeading : public BT::StatefulActionNode
    {
    public:
        StubDriveOnHeading(const std::string& name, const BT::NodeConfig& config)
            : BT::StatefulActionNode(name, config)
        {
        }

        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<double>("dist_to_travel"), BT::InputPort<double>("speed"),
                    BT::InputPort<double>("time_allowance"),
                    BT::InputPort<std::string>("server_name")};
        }

        BT::NodeStatus onStart() override
        {
            double distance = 0.0;
            double speed = 0.0;
            getInput("dist_to_travel", distance);
            getInput("speed", speed);
            const double seconds = std::abs(distance) / std::max(std::abs(speed), 0.01);
            _deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double>(seconds));
            std::printf("  (stub drive: %.2f m at %.2f m/s, %.1f s, no motion)\n", distance,
                        speed, seconds);
            std::fflush(stdout);
            return BT::NodeStatus::RUNNING;
        }

        BT::NodeStatus onRunning() override
        {
            return std::chrono::steady_clock::now() >= _deadline ? BT::NodeStatus::SUCCESS
                                                                 : BT::NodeStatus::RUNNING;
        }

        void onHalted() override {}

    private:
        std::chrono::steady_clock::time_point _deadline;
    };

    struct Options
    {
        std::string command;
        std::string name;  // run's tree, pose's pose
        std::optional<double> lift, tilt, jaw, duration, timeout;
        std::optional<int> attempts;
        std::string drive = "auto";
        std::optional<std::string> server, xml, dir;
        bool rearm = false;
        int groot_port = 1667;
        double start_delay = 0.0;  // seconds to connect Groot2 before the first tick
        double hold = 0.0;         // seconds Groot2 can still see the tree once it ends
    };

    std::optional<Options> parse(const std::vector<std::string>& args)
    {
        if (args.size() < 2)
            return std::nullopt;
        Options o;
        o.command = args[1];
        size_t i = 2;
        if ((o.command == "run" || o.command == "pose") && i < args.size() &&
            args[i].rfind("--", 0) != 0)
            o.name = args[i++];
        for (; i < args.size(); ++i)
        {
            const auto& flag = args[i];
            if (flag == "--rearm")
            {
                o.rearm = true;
                continue;
            }
            if (i + 1 >= args.size())
            {
                std::cerr << "missing a value for " << flag << "\n";
                return std::nullopt;
            }
            const auto& value = args[++i];
            try
            {
                if (flag == "--lift")
                    o.lift = std::stod(value);
                else if (flag == "--tilt")
                    o.tilt = std::stod(value);
                else if (flag == "--jaw")
                    o.jaw = std::stod(value);
                else if (flag == "--duration")
                    o.duration = std::stod(value);
                else if (flag == "--timeout")
                    o.timeout = std::stod(value);
                else if (flag == "--attempts")
                    o.attempts = std::stoi(value);
                else if (flag == "--groot-port")
                    o.groot_port = std::stoi(value);
                else if (flag == "--start-delay")
                    o.start_delay = std::stod(value);
                else if (flag == "--hold")
                    o.hold = std::stod(value);
                else if (flag == "--dir")
                    o.dir = value;
                else if (flag == "--drive" &&
                         (value == "auto" || value == "nav2" || value == "stub"))
                    o.drive = value;
                else if (flag == "--server")
                    o.server = value;
                else if (flag == "--xml")
                    o.xml = value;
                else
                {
                    std::cerr << "unknown option " << flag << " " << value << "\n";
                    return std::nullopt;
                }
            }
            catch (const std::exception&)
            {
                std::cerr << flag << " needs a number, got " << value << "\n";
                return std::nullopt;
            }
        }
        return o;
    }

    /// One MoveBucket step as its own tree; unset joints are left off, so they hold.
    /// Direct, unless @p duration makes it a timed move.
    std::string move_tree(std::optional<double> lift, std::optional<double> tilt,
                          std::optional<double> jaw, std::optional<double> duration = {})
    {
        std::ostringstream xml;
        xml << R"(<root BTCPP_format="4"><BehaviorTree ID="BucketCliMove"><MoveBucket)";
        if (lift)
            xml << " lift_deg=\"" << *lift << "\"";
        if (tilt)
            xml << " tilt_deg=\"" << *tilt << "\"";
        if (jaw)
            xml << " jaw_deg=\"" << *jaw << "\"";
        if (duration)
            xml << " duration_s=\"" << *duration << "\"";
        else
            xml << R"( timeout_s="{bucket_move_s}" attempts="{bucket_move_attempts}")";
        xml << R"( server_name="{bucket_action}"/>)"
            << "</BehaviorTree></root>";
        return xml.str();
    }

    namespace fs = std::filesystem;

    /// The bucket behaviours directory: --dir, else the source copy of the installed
    /// one. Under colcon's symlink-install every installed file links back to the
    /// source tree, which is where poses and trees must be saved to be kept.
    fs::path bucket_dir(const Options& options)
    {
        if (options.dir)
            return fs::absolute(*options.dir);
        const fs::path installed =
            fs::path(ament_index_cpp::get_package_share_directory("autonomy_bringup")) /
            "behavior_trees" / "bucket";
        std::error_code error;
        const auto poses = fs::canonical(installed / "bucket_poses.xml", error);
        return error ? installed : poses.parent_path();
    }

    /// Every *.xml in @p dir, sorted, so trees register in a stable order.
    std::vector<fs::path> xml_files(const fs::path& dir)
    {
        std::vector<fs::path> files;
        for (const auto& entry : fs::directory_iterator(dir))
            if (entry.is_regular_file() && entry.path().extension() == ".xml")
                files.push_back(entry.path());
        std::sort(files.begin(), files.end());
        return files;
    }

    /// Writes @p dir's Groot2 project: every tree file, and the models of the
    /// plugins registered on @p factory so Groot2 knows their ports.
    int write_project(const BT::BehaviorTreeFactory& factory, const fs::path& dir)
    {
        const std::string models = BT::writeTreeNodesModelXML(factory, false);
        const auto begin = models.find("<TreeNodesModel>");
        const auto end = models.find("</TreeNodesModel>");
        if (begin == std::string::npos || end == std::string::npos)
        {
            std::cerr << "could not generate the node models\n";
            return 1;
        }

        const fs::path project = dir / "bucket.btproj";
        std::ofstream out(project);
        out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
            << "<!-- Groot2 project for the bucket behaviours. Generated by `bucket_cli "
               "project`;\n     rerun it after adding a tree file outside Groot2. -->\n"
            << "<root BTCPP_format=\"4\" project_name=\"bucket\">\n";
        for (const auto& file : xml_files(dir))
            out << "    <include path=\"" << file.filename().string() << "\"/>\n";
        out << "    <!-- Description of Node Models (used by Groot) -->\n    "
            << models.substr(begin, end + std::string("</TreeNodesModel>").size() - begin)
            << "\n</root>\n";
        if (!out)
        {
            std::cerr << "could not write " << project << "\n";
            return 1;
        }
        std::cout << "wrote " << project.string() << "\n";
        return 0;
    }

    /// /bucket/operator_override as bucket_driver last latched it, or nothing if
    /// the driver is not running.
    std::optional<bool> operator_has_bucket(const rclcpp::Node::SharedPtr& node)
    {
        std::optional<bool> value;
        auto subscription = node->create_subscription<std_msgs::msg::Bool>(
            "/bucket/operator_override", rclcpp::QoS(1).reliable().transient_local(),
            [&value](const std_msgs::msg::Bool& msg)
            { value = msg.data; });
        const auto deadline = std::chrono::steady_clock::now() + 1s;
        while (!value && std::chrono::steady_clock::now() < deadline && rclcpp::ok())
            rclcpp::spin_some(node);
        return value;
    }

    int rearm(const rclcpp::Node::SharedPtr& node)
    {
        auto client = node->create_client<std_srvs::srv::Trigger>("/bucket/rearm");
        if (!client->wait_for_service(2s))
        {
            std::cerr << "/bucket/rearm is not available - is bucket_driver (the bucket "
                         "teleop) running?\n";
            return 1;
        }
        auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        if (rclcpp::spin_until_future_complete(node, future, 5s) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            std::cerr << "no answer from /bucket/rearm\n";
            return 1;
        }
        const auto response = future.get();
        std::cout << (response->success ? "re-armed: " : "refused: ") << response->message
                  << "\n";
        return response->success ? 0 : 1;
    }
}  // namespace

int main(int argc, char** argv)
{
    // Our own SIGINT handling: on Ctrl-C the tree must be halted - cancelling the
    // goal in flight - while the ROS context is still alive to send the cancel.
    rclcpp::InitOptions init_options;
    rclcpp::init(argc, argv, init_options, rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, [](int)
                { g_interrupted = true; });

    const auto options = parse(rclcpp::remove_ros_arguments(argc, argv));
    static const std::map<std::string, std::string> SUBTREES{
        {"prepare", "PrepareBucket"}, {"dig", "DigBucket"}, {"dump", "DumpBucket"}};
    static const std::set<std::string> OTHERS{"zero", "travel", "move", "run",
                                              "pose", "project", "rearm"};
    const bool known =
        options && (SUBTREES.count(options->command) || OTHERS.count(options->command));
    if (!known)
    {
        std::cerr << USAGE;
        rclcpp::shutdown();
        return 2;
    }

    auto node = std::make_shared<rclcpp::Node>("bucket_cli");
    if (options->command == "rearm")
    {
        const int code = rearm(node);
        rclcpp::shutdown();
        return code;
    }
    if (options->command == "move" && !options->lift && !options->tilt && !options->jaw)
    {
        std::cerr << "move needs at least one of --lift, --tilt, --jaw\n";
        rclcpp::shutdown();
        return 2;
    }
    if ((options->command == "run" || options->command == "pose") && options->name.empty())
    {
        std::cerr << options->command << " needs a " << (options->command == "run" ? "tree" : "pose")
                  << " name\n";
        rclcpp::shutdown();
        return 2;
    }
    const bool own_trees = options->command == "run" || options->command == "pose" ||
                           options->command == "project";
    const fs::path dir = bucket_dir(*options);
    if (own_trees && !fs::is_directory(dir))
    {
        std::cerr << "no bucket behaviours directory at " << dir << " - pass --dir\n";
        rclcpp::shutdown();
        return 1;
    }

    auto params = BucketMissionParams::declare(*node);
    const auto timeouts = BtTimeouts::declare(*node);
    if (options->timeout)
        params.move_s = *options->timeout;
    if (options->attempts)
        params.move_attempts = *options->attempts;
    if (options->server)
        params.action_name = *options->server;

    // The BT nodes build their clients on this one and spin it themselves.
    auto bt_node = std::make_shared<rclcpp::Node>("bucket_cli_bt");

    BT::BehaviorTreeFactory factory;
    try
    {
        // Everything mission.xml names must be registered for it to load at all,
        // even the nodes the bucket subtrees never build.
        for (const char* plugin : {"move_bucket_bt_node", "blind_jaw_bt_node",
                                   "request_zone_waypoint_bt_node",
                                   "nav2_navigate_to_pose_action_bt_node"})
            factory.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));

        // The project describes the real DriveOnHeading's ports, never the stub's.
        bool nav2_drive = options->drive == "nav2" || options->command == "project";
        if (options->drive == "auto" && (options->command == "dig" || options->command == "run"))
        {
            auto probe = rclcpp_action::create_client<nav2_msgs::action::DriveOnHeading>(
                bt_node, "drive_on_heading");
            nav2_drive = probe->wait_for_action_server(1s);
            if (!nav2_drive)
                std::cout << "drive_on_heading is not up: DriveOnHeading (the dig's creeps) "
                             "is stubbed, no motion. --drive nav2 to insist.\n";
        }
        if (nav2_drive)
            factory.registerFromPlugin(
                BT::SharedLibrary::getOSName("nav2_drive_on_heading_bt_node"));
        else
            factory.registerNodeType<StubDriveOnHeading>("DriveOnHeading");

        const auto xml = options->xml.value_or(
            ament_index_cpp::get_package_share_directory("autonomy_bringup") +
            "/behavior_trees/mission.xml");
        factory.registerBehaviorTreeFromFile(xml);
        // Your own trees and saved poses, so they can use mission.xml's subtrees too.
        if (own_trees)
            for (const auto& file : xml_files(dir))
                factory.registerBehaviorTreeFromFile(file.string());
    }
    catch (const std::exception& e)
    {
        std::cerr << "Could not load the bucket behaviours: " << e.what() << "\n";
        rclcpp::shutdown();
        return 1;
    }

    if (options->command == "project")
    {
        const int code = write_project(factory, dir);
        rclcpp::shutdown();
        return code;
    }
    // Only when the operator has it: re-arming an active controller is refused, and
    // with no bucket_driver running there is no operator to take it back from.
    if (options->rearm && operator_has_bucket(node).value_or(false) && rearm(node) != 0)
    {
        rclcpp::shutdown();
        return 1;
    }

    auto blackboard = BT::Blackboard::create();
    timeouts.apply(*blackboard, bt_node);
    params.apply(*blackboard);
    blackboard->set<std::string>("phase", "starting");

    std::string tree_id;
    if (const auto it = SUBTREES.find(options->command); it != SUBTREES.end())
        tree_id = it->second;
    else if (options->command == "run")
        tree_id = options->name;
    else if (options->command == "pose")
        tree_id = "Pose_" + options->name;
    else
    {
        tree_id = "BucketCliMove";
        if (options->command == "zero")
            factory.registerBehaviorTreeFromText(move_tree(0.0, 0.0, 0.0));
        else if (options->command == "travel")
            factory.registerBehaviorTreeFromText(
                move_tree(params.travel_lift_deg, params.travel_tilt_deg, params.travel_jaw_deg));
        else
            factory.registerBehaviorTreeFromText(
                move_tree(options->lift, options->tilt, options->jaw, options->duration));
    }

    BT::Tree tree;
    try
    {
        tree = factory.createTree(tree_id, blackboard);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Could not build " << tree_id << ": " << e.what() << "\n";
        rclcpp::shutdown();
        return 1;
    }

    // A port that will not bind (mission_bt_server already on it) costs the live
    // view, never the run.
    std::unique_ptr<BT::Groot2Publisher> groot;
    if (options->groot_port > 0)
    {
        try
        {
            groot = std::make_unique<BT::Groot2Publisher>(
                tree, static_cast<unsigned>(options->groot_port));
        }
        catch (const std::exception& e)
        {
            std::cerr << "Groot2 monitoring unavailable on port " << options->groot_port
                      << ": " << e.what() << "\n";
        }
    }

    std::cout << "Running " << tree_id << " on " << params.action_name;
    if (groot)
        std::cout << " (Groot2: monitor port " << options->groot_port << ")";
    std::cout << "\n"
              << std::flush;
    // Groot2 can only connect while a tree is up, and a short one is over before
    // anyone clicks Connect: give them a moment first, the bucket still.
    if (groot && options->start_delay > 0.0)
    {
        std::cout << "Connect Groot2's Monitor to port " << options->groot_port
                  << " now - starting in " << options->start_delay << " s\n"
                  << std::flush;
        if (!wait_unless_interrupted(options->start_delay))
        {
            std::cout << "Interrupted before starting - the bucket did not move\n";
            rclcpp::shutdown();
            return 130;
        }
    }
    std::string last_phase;
    auto status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING && rclcpp::ok() && !g_interrupted)
    {
        status = tree.tickOnce();
        std::string phase;
        if (blackboard->get("phase", phase) && phase != last_phase && phase != "starting")
        {
            std::cout << "  " << phase << "\n"
                      << std::flush;
            last_phase = phase;
        }
        tree.sleep(timeouts.bt_loop_duration);
    }

    int code = 0;
    if (g_interrupted)
    {
        // MoveBucket cancels its goal, so the controller holds where it is.
        tree.haltTree();
        std::cout << "Interrupted - the bucket holds where it is\n";
        code = 130;
    }
    else if (status == BT::NodeStatus::SUCCESS)
        std::cout << tree_id << " succeeded\n";
    else
    {
        std::cout << tree_id << " FAILED\n";
        code = 1;
    }
    // The tree and its publisher stay up, so Groot2 still shows how each node ended.
    if (groot && options->hold > 0.0 && !g_interrupted)
    {
        std::cout << "Holding the finished tree on Groot2 for " << options->hold
                  << " s (Ctrl-C to end)\n"
                  << std::flush;
        wait_unless_interrupted(options->hold);
    }
    rclcpp::shutdown();
    return code;
}
