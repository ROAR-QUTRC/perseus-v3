#pragma once

/// @file wiggle_bucket_bt_node.hpp
/// @brief BT.CPP leaf that wiggles each bucket axis out and back, the recovery for a
/// pose the bucket keeps missing.

#include <array>
#include <cstddef>
#include <optional>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

#include "mission_bt_server/mission_bt_server/move_bucket_bt_node.hpp"

namespace mission_bt_server
{

    /// @brief Moves each bucket axis in turn by its offset and back to where it
    /// started: lift out and back, then tilt, then jaw.
    ///
    /// For when MoveBucket gives up on a pose with not_reached - the bucket stuck just
    /// outside tolerance. Shaking each axis loose and then sending the pose again
    /// often gets it there. Use it in a Fallback, with the pose retried after it:
    ///
    ///   <Fallback>
    ///     <SubTree ID="Pose_X"/>
    ///     <Sequence>
    ///       <SubTree ID="Wiggle"/>
    ///       <SubTree ID="Pose_X"/>
    ///     </Sequence>
    ///   </Fallback>
    ///
    /// lift_deg, tilt_deg and jaw_deg are signed offsets in the bucket joint
    /// conventions (lift positive lowers, tilt positive dumps, jaw positive opens), so
    /// the sign is which way the axis goes first. 0 leaves that axis alone. The start
    /// angles are read from /joint_states just before moving, and "back" returns to
    /// them, whether or not the outward move got all the way.
    ///
    /// Each move is a direct MoveBucket of one joint, the other two held. A move that
    /// misses its tolerance is logged and the wiggle carries on: the pose retried
    /// afterwards decides whether the recovery worked. tolerance_deg is always sent,
    /// because the controller's own tilt tolerance (0.5 rad, about 29 deg) would let a
    /// 5 deg tilt move report success without the bucket moving at all.
    ///
    /// Everything else is MoveBucket's: it fails at once on an operator override, a
    /// missing controller or a rejected goal, and halting it leaves the bucket holding
    /// where it is.
    class WiggleBucketBtNode : public MoveBucketBtNode
    {
    public:
        WiggleBucketBtNode(const std::string& name, const BT::NodeConfiguration& conf);

        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;

    private:
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_subscription;
        // Latest lift, tilt and jaw angles, radians, and how many /joint_states
        // messages have come in. Written inside _spin(), on the tree's thread.
        std::array<std::optional<double>, 3> _angles;
        std::size_t _joint_messages{0};
    };

}  // namespace mission_bt_server
