// Holds bucket_linkage.hpp to the numbers description/scripts/bucket_ram_follower.py
// produces, so the C++ broadcaster and the Python slider viewer pose the rams the
// same way. If you change the linkage, change both and regenerate the table:
//
//   cd software/ros_ws/src/description/scripts && python3 - <<'EOF'
//   import importlib.util, math, sys, types
//   for m in ["rclpy", "rclpy.node", "rclpy.qos", "sensor_msgs", "sensor_msgs.msg",
//             "std_msgs", "std_msgs.msg"]:
//       sys.modules.setdefault(m, types.ModuleType(m))
//   sys.modules["rclpy.node"].Node = object
//   q = sys.modules["rclpy.qos"]
//   q.DurabilityPolicy = q.QoSProfile = q.ReliabilityPolicy = object
//   sys.modules["sensor_msgs.msg"].JointState = object
//   sys.modules["std_msgs.msg"].Float64 = sys.modules["std_msgs.msg"].String = object
//   spec = importlib.util.spec_from_file_location("f", "bucket_ram_follower.py")
//   f = importlib.util.module_from_spec(spec); spec.loader.exec_module(f)
//   for lift, tilt, jaw in [(0, 0, 0), (25, -20, 0), (32, 28, 0), (20, 20, 36),
//                           (71.4, 28.098, 47.991), (0, -91.673, 0)]:
//       v = f.ram_values(*map(math.radians, (lift, tilt, jaw)))
//       print([v[k] for k in ("lift_angle", "lift_extend", "lift_angle", "lift_extend",
//              "tilt_angle", "tilt_extend", "jaw_angle", "jaw_extend", "jaw_angle",
//              "jaw_extend")])
//   EOF

#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "bucket_linkage/bucket_linkage.hpp"

namespace
{
    struct Case
    {
        double lift_deg;
        double tilt_deg;
        double jaw_deg;
        std::array<double, 10> expected;
    };

    // Zero, the travel pose, the dig pose, the dump pose, every joint at its
    // upper limit, and the tilt at its lower limit.
    const Case CASES[] = {
        {0, 0, 0, {0.367738506, 0.274769323, 0.367738506, 0.274769323, 0.193544461, 0.203321318, -0.000004458, 0.000000000, -0.000004458, 0.000000000}},
        {25, -20, 0, {-0.039804805, 0.172254027, -0.039804805, 0.172254027, 0.224411136, 0.161047725, -0.000004458, 0.000000000, -0.000004458, 0.000000000}},
        {32, 28, 0, {-0.167796373, 0.143107619, -0.167796373, 0.143107619, 0.120686450, 0.249874701, -0.000004458, 0.000000000, -0.000004458, 0.000000000}},
        {20, 20, 36, {0.047324302, 0.193192070, 0.047324302, 0.193192070, 0.144143157, 0.238464573, 0.020689436, -0.038771849, 0.020689436, -0.038771849}},
        {71.4, 28.098, 47.991, {-1.068104760, 0.010486228, -1.068104760, 0.010486228, 0.120388703, 0.250000000, 0.063078077, -0.050000000, 0.063078077, -0.050000000}},
        {0, -91.673, 0, {0.367738506, 0.274769323, 0.367738506, 0.274769323, 0.068807689, 0.024668082, -0.000004458, 0.000000000, -0.000004458, 0.000000000}},
    };

    double radians(double degrees) { return degrees * M_PI / 180.0; }
}  // namespace

TEST(BucketLinkage, MatchesPythonFollower)
{
    using payloads::bucket_linkage::RAM_JOINTS;
    for (const auto& c : CASES)
    {
        const auto actual = payloads::bucket_linkage::ram_positions(
            radians(c.lift_deg), radians(c.tilt_deg), radians(c.jaw_deg));
        for (size_t i = 0; i < actual.size(); ++i)
        {
            EXPECT_NEAR(actual[i], c.expected[i], 1e-8)
                << RAM_JOINTS[i] << " at lift " << c.lift_deg << ", tilt " << c.tilt_deg
                << ", jaw " << c.jaw_deg;
        }
    }
}

TEST(BucketLinkage, ExtensionsStayInStroke)
{
    using namespace payloads::bucket_linkage;
    for (double lift = 0.0; lift <= 72.0; lift += 4.0)
        for (double tilt = -92.0; tilt <= 29.0; tilt += 4.0)
            for (double jaw = 0.0; jaw <= 48.0; jaw += 4.0)
            {
                const auto p = ram_positions(radians(lift), radians(tilt), radians(jaw));
                EXPECT_GE(p[1], 0.0);
                EXPECT_LE(p[1], lift_ram_stroke());
                EXPECT_GE(p[5], 0.0);
                EXPECT_LE(p[5], RAM_STROKE);
                EXPECT_GE(p[7], -JAW_STROKE);
                EXPECT_LE(p[7], 0.0);
            }
}
