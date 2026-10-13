/// @file bucket_mission_params.cpp
/// @brief Declarations and blackboard population shared by mission_bt_server and bucket_cli.

#include "mission_bt_server/mission_bt_server/bucket_mission_params.hpp"

#include <algorithm>

namespace mission_bt_server
{

    BtTimeouts BtTimeouts::declare(rclcpp::Node& node)
    {
        // Matches nav2_behavior_tree::BtActionServer's own defaults (bt_loop_duration
        // 10ms, default_server_timeout/default_cancel_timeout 20s,
        // wait_for_service_timeout 1s) so this tree behaves the same as any other
        // nav2 BT with respect to how fast it ticks and how long it tolerates a slow
        // server ack - only the overall navigation itself is unbounded by these.
        BtTimeouts t;
        t.bt_loop_duration =
            std::chrono::milliseconds(node.declare_parameter<int>("bt_loop_duration_ms", 10));
        t.server_timeout = std::chrono::milliseconds(
            node.declare_parameter<int>("default_server_timeout_ms", 20000));
        t.cancel_timeout = std::chrono::milliseconds(
            node.declare_parameter<int>("default_cancel_timeout_ms", 20000));
        t.wait_for_service_timeout = std::chrono::milliseconds(
            node.declare_parameter<int>("wait_for_service_timeout_ms", 1000));
        return t;
    }

    void BtTimeouts::apply(BT::Blackboard& blackboard,
                           const rclcpp::Node::SharedPtr& bt_node) const
    {
        blackboard.set<rclcpp::Node::SharedPtr>("node", bt_node);
        blackboard.set<std::chrono::milliseconds>("server_timeout", server_timeout);
        blackboard.set<std::chrono::milliseconds>("cancel_timeout", cancel_timeout);
        blackboard.set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration);
        blackboard.set<std::chrono::milliseconds>("wait_for_service_timeout",
                                                  wait_for_service_timeout);
    }

    BucketMissionParams BucketMissionParams::declare(rclcpp::Node& node)
    {
        BucketMissionParams p;
        // PrepareBucket (StartMission.prepare_bucket): the bucket controller's action,
        // and the pose it leaves the bucket in for driving. The default pose keeps the
        // bucket out of the MID-360's and D455's view of the ground from 2 m ahead
        // while leaving ~0.17 m of ground clearance; see mission.xml.
        p.action_name = node.declare_parameter<std::string>("bucket_action_name",
                                                            DEFAULT_BUCKET_ACTION);
        p.travel_lift_deg = node.declare_parameter<double>("bucket_travel_lift_deg", 25.0);
        p.travel_tilt_deg = node.declare_parameter<double>("bucket_travel_tilt_deg", -20.0);
        p.travel_jaw_deg = node.declare_parameter<double>("bucket_travel_jaw_deg", 0.0);
        // Per move. Generous: the controller has to get there within this plus its
        // 3 s goal_time tolerance, from wherever the bucket was left.
        p.move_s = node.declare_parameter<double>("bucket_move_s", 10.0);

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
        p.dig_tilt_deg = node.declare_parameter<double>("dig_tilt_deg", 28.0);
        p.dig_lift_deg = node.declare_parameter<double>("dig_lift_deg", 32.0);
        p.dig_push_m = node.declare_parameter<double>("dig_push_m", 0.20);
        p.dig_curl_tilt_deg = node.declare_parameter<double>("dig_curl_tilt_deg", -28.0);
        p.dig_carry_m = node.declare_parameter<double>("dig_carry_m", 0.40);
        p.dig_carry_lift_deg = node.declare_parameter<double>("dig_carry_lift_deg", 0.0);
        p.dig_speed = node.declare_parameter<double>("dig_speed", 0.08);
        return p;
    }

    void BucketMissionParams::apply(BT::Blackboard& blackboard) const
    {
        blackboard.set<std::string>("bucket_action", action_name);
        blackboard.set<double>("bucket_travel_lift_deg", travel_lift_deg);
        blackboard.set<double>("bucket_travel_tilt_deg", travel_tilt_deg);
        blackboard.set<double>("bucket_travel_jaw_deg", travel_jaw_deg);
        blackboard.set<double>("bucket_move_s", move_s);
        blackboard.set<double>("dig_tilt_deg", dig_tilt_deg);
        blackboard.set<double>("dig_lift_deg", dig_lift_deg);
        blackboard.set<double>("dig_push_m", dig_push_m);
        blackboard.set<double>("dig_curl_tilt_deg", dig_curl_tilt_deg);
        blackboard.set<double>("dig_carry_m", dig_carry_m);
        blackboard.set<double>("dig_carry_lift_deg", dig_carry_lift_deg);
        blackboard.set<double>("dig_speed", dig_speed);
        // The carry's drive and lift start together; the lift gets at least
        // bucket_move_s, since 32 -> 0 deg in a 5 s creep would outrun the rams.
        const double carry_s = dig_carry_m / std::max(dig_speed, 0.01);
        blackboard.set<double>("dig_carry_s", std::max(carry_s, move_s));
        // DriveOnHeading gives up after time_allowance; leave room for the ramp-up.
        blackboard.set<double>("dig_push_allowance_s",
                               dig_push_m / std::max(dig_speed, 0.01) + 10.0);
        blackboard.set<double>("dig_carry_allowance_s", carry_s + 10.0);
    }

}  // namespace mission_bt_server
