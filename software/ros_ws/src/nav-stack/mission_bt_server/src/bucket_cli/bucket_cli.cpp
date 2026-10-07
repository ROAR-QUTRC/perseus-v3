/// @file bucket_cli.cpp
/// @brief Runs the mission's bucket behaviours from the command line.
///
/// The same subtrees, plugins and parameter values mission_bt_server runs
/// (mission.xml's PrepareBucket, DigBucket and DumpBucket, and single MoveBucket
/// steps), without the rest of the mission stack - so the bucket can be exercised
/// on the bench, on the rover, or against payloads' rover_can_sim.py on vcan.
///
///   ros2 run mission_bt_server bucket_cli dump
///   ros2 run mission_bt_server bucket_cli move --lift 30 --tilt -10
///   ros2 run mission_bt_server bucket_cli dig --drive stub
///   ros2 run mission_bt_server bucket_cli rearm
///
/// Mission values are ROS parameters, as on mission_bt_server:
///   ros2 run mission_bt_server bucket_cli dig --ros-args -p dig_lift_deg:=30.0

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/utils/shared_library.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <map>
#include <nav2_msgs/action/drive_on_heading.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sstream>
#include <std_srvs/srv/trigger.hpp>
#include <string>
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
  rearm      hand the bucket back from the operator (/bucket/rearm)

options:
  --lift DEG, --tilt DEG, --jaw DEG   move's targets
  --duration S       how long each move takes (default: bucket_move_s, 10)
  --drive MODE       dig's creeps: nav2 (behavior_server's drive_on_heading),
                     stub (pretend: wait out the drive's time, no motion), or
                     auto (nav2 if it answers within 1 s, else stub) [auto]
  --server NAME      the bucket controller's FollowJointTrajectory action
  --xml PATH         mission tree (default: autonomy_bringup's mission.xml)
)";

    std::atomic<bool> g_interrupted{false};

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
        std::optional<double> lift, tilt, jaw, duration;
        std::string drive = "auto";
        std::optional<std::string> server, xml;
    };

    std::optional<Options> parse(const std::vector<std::string>& args)
    {
        if (args.size() < 2)
            return std::nullopt;
        Options o;
        o.command = args[1];
        for (size_t i = 2; i < args.size(); ++i)
        {
            const auto& flag = args[i];
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
    std::string move_tree(std::optional<double> lift, std::optional<double> tilt,
                          std::optional<double> jaw)
    {
        std::ostringstream xml;
        xml << R"(<root BTCPP_format="4"><BehaviorTree ID="BucketCliMove"><MoveBucket)";
        if (lift)
            xml << " lift_deg=\"" << *lift << "\"";
        if (tilt)
            xml << " tilt_deg=\"" << *tilt << "\"";
        if (jaw)
            xml << " jaw_deg=\"" << *jaw << "\"";
        xml << R"( duration_s="{bucket_move_s}" server_name="{bucket_action}"/>)"
            << "</BehaviorTree></root>";
        return xml.str();
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
    const bool known = options && (SUBTREES.count(options->command) ||
                                   options->command == "zero" || options->command == "travel" ||
                                   options->command == "move" || options->command == "rearm");
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

    auto params = BucketMissionParams::declare(*node);
    const auto timeouts = BtTimeouts::declare(*node);
    if (options->duration)
        params.move_s = *options->duration;
    if (options->server)
        params.action_name = *options->server;

    // The BT nodes build their clients on this one and spin it themselves.
    auto bt_node = std::make_shared<rclcpp::Node>("bucket_cli_bt");

    BT::BehaviorTreeFactory factory;
    try
    {
        // Everything mission.xml names must be registered for it to load at all,
        // even the nodes the bucket subtrees never build.
        for (const char* plugin : {"move_bucket_bt_node", "request_zone_waypoint_bt_node",
                                   "nav2_navigate_to_pose_action_bt_node"})
            factory.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));

        bool nav2_drive = options->drive == "nav2";
        if (options->drive == "auto" && options->command == "dig")
        {
            auto probe = rclcpp_action::create_client<nav2_msgs::action::DriveOnHeading>(
                bt_node, "drive_on_heading");
            nav2_drive = probe->wait_for_action_server(1s);
            if (!nav2_drive)
                std::cout << "drive_on_heading is not up: the dig's creeps are stubbed "
                             "(no motion). --drive nav2 to insist.\n";
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
    }
    catch (const std::exception& e)
    {
        std::cerr << "Could not load the bucket behaviours: " << e.what() << "\n";
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
                move_tree(options->lift, options->tilt, options->jaw));
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

    std::cout << "Running " << tree_id << " on " << params.action_name << "\n"
              << std::flush;
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
    rclcpp::shutdown();
    return code;
}
