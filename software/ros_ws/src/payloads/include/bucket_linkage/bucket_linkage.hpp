#pragma once

/// @file bucket_linkage.hpp
/// @brief Closed-form ram poses for the excavation bucket's linkage.
///
/// The ten ram joints in bucket.urdf.xacro (two lift rams, one tilt ram, two jaw
/// rams, each a pitch and an extension) are not free: each is a function of the
/// lift, tilt and jaw angles. This is a line-for-line port of ram_values() in
/// description/scripts/bucket_ram_follower.py, which keeps the slider viewer
/// working; test/test_bucket_linkage.cpp holds the two to the same numbers. Change
/// one and you must change the other. The geometry is documented there.

#include <algorithm>
#include <array>
#include <cmath>
#include <string_view>

namespace payloads::bucket_linkage
{
    // Linkage constants, mirroring bucket.urdf.xacro.
    inline constexpr double LIFT_R = 0.59712;       // arm pivot -> lift bracket
    inline constexpr double LIFT_BASE_X = 0.00004;  // ram pivot, relative to the arm pivot
    inline constexpr double LIFT_BASE_Z = -0.24001;
    inline constexpr double LIFT_PHASE = -0.016763;  // the bracket sits this far below the arm line
    inline constexpr double RAM_RETRACTED = 0.365;   // pin-to-pin, as drawn in CAD
    inline constexpr double RAM_STROKE = 0.250;

    // Tilt, solved in the arm frame.
    inline constexpr double TILT_ANCHOR_X = 0.18674;
    inline constexpr double TILT_ANCHOR_Z = 0.01399;
    inline constexpr double TILT_PIVOT_X = 0.70004;
    inline constexpr double TILT_PIVOT_Z = -0.00001;
    inline constexpr double TILT_PIN_X = 0.04441;
    inline constexpr double TILT_PIN_Z = 0.12331;

    // Jaw rams, in the bucket frame (x forward, z up).
    inline constexpr double JAW_HINGE_X = 0.07645;
    inline constexpr double JAW_HINGE_Z = 0.15894;
    inline constexpr double JAW_BASE_X = 0.00974;  // barrel pin, on the REAR half
    inline constexpr double JAW_BASE_Z = -0.03206;
    inline constexpr double JAW_PIN0_X = 0.0154;  // rod pin at jaw angle 0, on the FRONT half
    inline constexpr double JAW_PIN0_Z = 0.17467;
    inline constexpr double JAW_STROKE = 0.05;
    inline constexpr double JAW_PHI0 = -1.54342;  // the pitch joint's rest angle

    // Bracket angle at lift zero: the arms are level at the top of travel.
    inline constexpr double LIFT_A_ZERO = LIFT_PHASE;

    /// The ram's extension with the arms level, as lift_ram_stroke in bucket.urdf.xacro.
    inline double lift_ram_stroke()
    {
        return std::hypot(LIFT_R * std::cos(LIFT_A_ZERO) - LIFT_BASE_X,
                          LIFT_R * std::sin(LIFT_A_ZERO) - LIFT_BASE_Z) -
               RAM_RETRACTED;
    }

    inline double jaw_len0()
    {
        return std::hypot(JAW_PIN0_X - JAW_BASE_X, JAW_PIN0_Z - JAW_BASE_Z);
    }

    struct RamPose
    {
        double angle;
        double extension;
    };

    /// Ram angle in the frame and extension, for a lift joint angle.
    inline RamPose lift_ram(double q_lift)
    {
        const double a = LIFT_A_ZERO + q_lift;
        const double vx = LIFT_R * std::cos(a) - LIFT_BASE_X;
        const double vz = LIFT_R * std::sin(a) - LIFT_BASE_Z;
        return {std::atan2(vz, vx), std::hypot(vx, vz) - RAM_RETRACTED};
    }

    /// Ram angle in the arm frame and extension, for a tilt joint angle.
    inline RamPose tilt_ram(double q_tilt)
    {
        const double c = std::cos(q_tilt);
        const double s = std::sin(q_tilt);
        // the bucket-side pin, swung into the arm frame by the tilt angle
        const double px = TILT_PIVOT_X + c * TILT_PIN_X - s * TILT_PIN_Z;
        const double pz = TILT_PIVOT_Z + s * TILT_PIN_X + c * TILT_PIN_Z;
        const double vx = px - TILT_ANCHOR_X;
        const double vz = pz - TILT_ANCHOR_Z;
        return {std::atan2(vz, vx), std::hypot(vx, vz) - RAM_RETRACTED};
    }

    /// Jaw-ram pitch (relative to its rest angle) and extension (0 = extended,
    /// -stroke = retracted), for a jaw joint angle.
    inline RamPose jaw_ram(double q_jaw)
    {
        const double c = std::cos(q_jaw);
        const double s = std::sin(q_jaw);
        const double dx = JAW_PIN0_X - JAW_HINGE_X;
        const double dz = JAW_PIN0_Z - JAW_HINGE_Z;
        // right-handed about +y: (x, z) -> (x c + z s, -x s + z c)
        const double px = JAW_HINGE_X + dx * c + dz * s;
        const double pz = JAW_HINGE_Z - dx * s + dz * c;
        const double vx = px - JAW_BASE_X;
        const double vz = pz - JAW_BASE_Z;
        return {std::atan2(-vz, vx) - JAW_PHI0, std::hypot(vx, vz) - jaw_len0()};
    }

    /// The ten ram joints, in the order ram_positions() fills them. Matches
    /// RAM_JOINTS in bucket_ram_follower.py, flattened lift, tilt, jaw.
    inline constexpr std::array<std::string_view, 10> RAM_JOINTS{
        "bucket_ram_lift_left_pitch_joint",
        "bucket_ram_lift_left_extend_joint",
        "bucket_ram_lift_right_pitch_joint",
        "bucket_ram_lift_right_extend_joint",
        "bucket_ram_tilt_pitch_joint",
        "bucket_ram_tilt_extend_joint",
        "bucket_ram_jaw_right_pitch_joint",
        "bucket_ram_jaw_right_extend_joint",
        "bucket_ram_jaw_left_pitch_joint",
        "bucket_ram_jaw_left_extend_joint",
    };

    /// Angle and clamped extension of every ram, for the three driven joints, in
    /// RAM_JOINTS order. Both rams of a pair move together.
    inline std::array<double, 10> ram_positions(double q_lift, double q_tilt, double q_jaw)
    {
        // The kinematics were derived with lift positive up, tilt positive curl and
        // jaw positive closing; the URDF joints count the other way.
        const auto lift = lift_ram(-q_lift);
        const auto tilt = tilt_ram(-q_tilt);
        const auto jaw = jaw_ram(-q_jaw);
        const double lift_extend = std::clamp(lift.extension, 0.0, lift_ram_stroke());
        const double tilt_extend = std::clamp(tilt.extension, 0.0, RAM_STROKE);
        const double jaw_extend = std::clamp(jaw.extension, -JAW_STROKE, 0.0);
        return {
            lift.angle,
            lift_extend,
            lift.angle,
            lift_extend,
            tilt.angle,
            tilt_extend,
            jaw.angle,
            jaw_extend,
            jaw.angle,
            jaw_extend,
        };
    }
}  // namespace payloads::bucket_linkage
