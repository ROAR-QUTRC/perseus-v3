#pragma once

/// @file bucket_mission_params.hpp
/// @brief The parameters and blackboard entries mission.xml's bucket subtrees read,
/// shared by mission_bt_server and bucket_cli so both run the same behaviours with
/// the same values.

#include <behaviortree_cpp/blackboard.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace mission_bt_server
{

    /// @brief The bucket controller's FollowJointTrajectory action, when nothing
    /// overrides it: the bucket's trajectory controller under the rover's one
    /// controller_manager (perseus.launch.py payload:=bucket).
    inline constexpr const char* DEFAULT_BUCKET_ACTION =
        "/bucket_trajectory_controller/follow_joint_trajectory";

    /// @brief Blackboard entries every nav2_behavior_tree BT node reads in its own
    /// constructor (BtServiceNode, BtActionNode) - see bt_action_server_impl.hpp,
    /// which this replicates without the LifecycleNode it comes attached to.
    struct BtTimeouts
    {
        std::chrono::milliseconds bt_loop_duration;
        std::chrono::milliseconds server_timeout;
        std::chrono::milliseconds cancel_timeout;
        std::chrono::milliseconds wait_for_service_timeout;

        /// @brief Declares them on @p node, with nav2's own defaults.
        static BtTimeouts declare(rclcpp::Node& node);

        /// @brief Sets them, and "node" (what the BT nodes build clients on).
        void apply(BT::Blackboard& blackboard, const rclcpp::Node::SharedPtr& bt_node) const;
    };

    /// @brief PrepareBucket's travel pose and DigBucket's pose, creeps and carry,
    /// as mission.xml reads them off the blackboard.
    struct BucketMissionParams
    {
        std::string action_name;
        double travel_lift_deg;
        double travel_tilt_deg;
        double travel_jaw_deg;
        double move_s;

        double dig_tilt_deg;
        double dig_lift_deg;
        double dig_push_m;
        double dig_curl_tilt_deg;
        double dig_carry_m;
        double dig_carry_lift_deg;
        double dig_speed;

        /// @brief Declares them on @p node; see the definition for what each is.
        static BucketMissionParams declare(rclcpp::Node& node);

        /// @brief Sets every bucket_* and dig_* entry, including the derived carry
        /// time and creep allowances.
        void apply(BT::Blackboard& blackboard) const;
    };

}  // namespace mission_bt_server
