#!/usr/bin/env python3
"""
Copied from perseus_simulation's scripts/bucket_ram_follower.py (branch bucket-urdf).
Only the viewer mode (viewer_mode:=true) is used in this repo, started by
view_perseus.launch.py for payload:=bucket; the Gazebo mode publishes controller
commands that nothing here consumes.

Drive the excavation bucket's rams so they follow the linkage: the three 250 mm
rams (two lift, one tilt) and the two 50 mm jaw rams.

Why this node exists
--------------------
The rams close two kinematic loops, and nothing in Gazebo Harmonic can express
that on its own:

  * A real closed loop is refused by the physics backend -
    "Asked to create a joint between links [x] as parent and [y] as child, but
    the child link already has a parent joint" (gz-physics-dartsim).
  * URDF ``<mimic>`` does convert into SDF ``<mimic>``, but dartsim does not
    enforce it - the dependent joint sits frozen at zero.
  * And ``<mimic>`` is affine, whereas both the ram angle and its extension are
    nonlinear in the driving joint. The best affine fit still misses the rod-end
    pin by 30 mm mid-travel.

So the relations are solved here in closed form, exactly, and fed to the Gazebo
position controllers declared in ``bucket.urdf.xacro``.

Two modes
---------
* Simulation (default): reads /joint_states from Gazebo and drives the ram
  position controllers over the six /bucket/ram_*/cmd_* topics.
* Viewer (``viewer_mode:=true``): there is no Gazebo, only a joint-state slider
  GUI. The GUI's output is routed to ``input_topic`` (default
  /joint_states_raw); this node overwrites the ten ram joints with the exact
  values and re-publishes the lot on /joint_states for robot_state_publisher.
  That way moving the lift, tilt or jaw slider moves the rams too. It also
  republishes the description on /robot_description_sliders with the ram joints
  fixed, so the slider GUI shows only the joints you can actually move.

The rams are cosmetic: their mass is already lumped onto the frame, arm and
bucket, and they have no collision geometry and no gravity. If this node is not
running they simply hold their last pose.

Geometry (all measured from the Onshape export, see perseus_description/cad/bucket)
----------------------------------------------------------------------------
Lift, in the frame's x-z plane, with the arm pivot at the origin:

    bracket   B = r * (cos a, sin a),  a = a_ext + q_lift
    ram base  P = (0, -d)
    ram vector V = B - P  ->  angle = atan2(Vz, Vx), extension = |V| - L_ret

Jaw, in the bucket's x-z plane: the rod pin is on the FRONT half, so it swings
about the jaw hinge by the jaw angle while the barrel pin stays put on the REAR
half. Both jaw actuators are planar and identical in x-z, so one angle and one
extension drive both sides.

Tilt, in the arm's x-z plane: the barrel is bolted to the plate on the arms, so
the anchor is fixed there, and the pin it drives is on the bucket - it has to be
swung into the arm frame by the tilt angle before the same subtraction.
"""

import math
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

# ── Linkage constants, mirroring bucket.urdf.xacro ──────────────────────────
LIFT_R = 0.59712  # arm pivot -> lift bracket
LIFT_D = 0.24000  # arm pivot above the lift-ram pivot
LIFT_BASE_X = 0.00004  # ram pivot, relative to the arm pivot
LIFT_BASE_Z = -0.24001
RAM_RETRACTED = 0.365  # pin-to-pin, as drawn in CAD
RAM_STROKE = 0.250

# Tilt, solved in the ARM frame: the ram's barrel is bolted to the plate on the
# arms, and its rod reaches forward to a pin on the bucket. So the anchor is
# fixed in the arm and the target swings with the bucket.
TILT_ANCHOR_X = 0.18674  # Actuator_Bracket_4, in the bucket_arm frame
TILT_ANCHOR_Z = 0.01399
TILT_PIVOT_X = 0.70004  # bucket pivot, in the bucket_arm frame
TILT_PIVOT_Z = -0.00001
TILT_PIN_X = 0.04441  # ram pin, in the bucket frame
TILT_PIN_Z = 0.12331

# Jaw rams, in the bucket frame (x forward, z up). Pins are the true pin lines,
# not the mate-connector origins - see "Jaw rams" in bucket.urdf.xacro.
JAW_HINGE = (0.07645, 0.15894)
JAW_BASE = (0.00974, -0.03206)  # barrel pin, on the REAR half
JAW_PIN0 = (0.0154, 0.17467)  # rod pin at jaw angle 0, on the FRONT half
JAW_STROKE = 0.05
JAW_PHI0 = -1.54342  # the pitch joint's rest angle (its origin rpy)
JAW_LEN0 = math.hypot(JAW_PIN0[0] - JAW_BASE[0], JAW_PIN0[1] - JAW_BASE[1])

LIFT_A_EXT = math.asin(
    ((RAM_RETRACTED + RAM_STROKE) ** 2 - LIFT_R**2 - LIFT_D**2) / (2 * LIFT_R * LIFT_D)
)


def lift_ram(q_lift):
    """Ram angle in the frame and extension, for a lift joint angle."""
    a = LIFT_A_EXT + q_lift
    vx = LIFT_R * math.cos(a) - LIFT_BASE_X
    vz = LIFT_R * math.sin(a) - LIFT_BASE_Z
    return math.atan2(vz, vx), math.hypot(vx, vz) - RAM_RETRACTED


def tilt_ram(q_tilt):
    """Ram angle in the arm frame and extension, for a tilt joint angle."""
    c, s = math.cos(q_tilt), math.sin(q_tilt)
    # the bucket-side pin, swung into the arm frame by the tilt angle
    px = TILT_PIVOT_X + c * TILT_PIN_X - s * TILT_PIN_Z
    pz = TILT_PIVOT_Z + s * TILT_PIN_X + c * TILT_PIN_Z
    vx, vz = px - TILT_ANCHOR_X, pz - TILT_ANCHOR_Z
    return math.atan2(vz, vx), math.hypot(vx, vz) - RAM_RETRACTED


def jaw_ram(q_jaw):
    """Jaw-ram pitch (relative to its rest angle) and extension (0 = extended,
    -stroke = retracted), for a jaw joint angle."""
    c, s = math.cos(q_jaw), math.sin(q_jaw)
    dx, dz = JAW_PIN0[0] - JAW_HINGE[0], JAW_PIN0[1] - JAW_HINGE[1]
    # right-handed about +y: (x, z) -> (x c + z s, -x s + z c)
    px = JAW_HINGE[0] + dx * c + dz * s
    pz = JAW_HINGE[1] - dx * s + dz * c
    vx, vz = px - JAW_BASE[0], pz - JAW_BASE[1]
    return math.atan2(-vz, vx) - JAW_PHI0, math.hypot(vx, vz) - JAW_LEN0


# the ten ram joints, in the order ram_values() returns them per ram group
RAM_JOINTS = {
    "lift": (
        "ram_lift_left_pitch_joint",
        "ram_lift_left_extend_joint",
        "ram_lift_right_pitch_joint",
        "ram_lift_right_extend_joint",
    ),
    "tilt": ("ram_tilt_pitch_joint", "ram_tilt_extend_joint"),
    "jaw": (
        "ram_jaw_right_pitch_joint",
        "ram_jaw_right_extend_joint",
        "ram_jaw_left_pitch_joint",
        "ram_jaw_left_extend_joint",
    ),
}


def ram_values(q_lift, q_tilt, q_jaw):
    """Angle and clamped extension of every ram, for the three driven joints."""
    la, ls = lift_ram(q_lift)
    ta, ts = tilt_ram(q_tilt)
    ja, js = jaw_ram(q_jaw)
    return {
        "lift_angle": la,
        "lift_extend": min(max(ls, 0.0), RAM_STROKE),
        "tilt_angle": ta,
        "tilt_extend": min(max(ts, 0.0), RAM_STROKE),
        "jaw_angle": ja,
        "jaw_extend": min(max(js, -JAW_STROKE), 0.0),
    }


class BucketRamFollower(Node):
    def __init__(self):
        super().__init__("bucket_ram_follower")
        self.declare_parameter("max_rate_hz", 30.0)
        self._min_dt = 1.0 / float(self.get_parameter("max_rate_hz").value)
        self._last = None

        self.declare_parameter("viewer_mode", False)
        self.declare_parameter("input_topic", "/joint_states_raw")
        self.declare_parameter("output_topic", "/joint_states")
        self._viewer = bool(self.get_parameter("viewer_mode").value)

        if self._viewer:
            self._out = self.create_publisher(
                JointState, self.get_parameter("output_topic").value, 10
            )
            self.create_subscription(
                JointState,
                self.get_parameter("input_topic").value,
                self._on_raw_states,
                10,
            )
            # The slider GUI offers one slider per movable joint, and the ten ram
            # joints are not free - they are functions of lift, tilt and jaw. So
            # give the GUI a copy of the description in which they are FIXED (a
            # fixed joint gets no slider). robot_state_publisher keeps the real
            # one, so the rams still move.
            latched = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self._desc_out = self.create_publisher(
                String, "robot_description_sliders", latched
            )
            self.create_subscription(
                String, "robot_description", self._on_description, latched
            )
            self.get_logger().info("bucket_ram_follower ready (viewer mode)")
            return

        self._pub = {
            name: self.create_publisher(Float64, topic, 10)
            for name, topic in (
                ("lift_angle", "/bucket/ram_lift/cmd_angle"),
                ("lift_extend", "/bucket/ram_lift/cmd_extend"),
                ("tilt_angle", "/bucket/ram_tilt/cmd_angle"),
                ("tilt_extend", "/bucket/ram_tilt/cmd_extend"),
                ("jaw_angle", "/bucket/ram_jaw/cmd_angle"),
                ("jaw_extend", "/bucket/ram_jaw/cmd_extend"),
            )
        }
        self.create_subscription(JointState, "/joint_states", self._on_states, 10)
        self.get_logger().info("bucket_ram_follower ready")

    def _on_description(self, msg):
        """Viewer mode: republish the description with the ram joints fixed."""
        ram_names = {n for group in RAM_JOINTS.values() for n in group}
        root = ET.fromstring(msg.data)
        for joint in root.iter("joint"):
            if joint.get("name") in ram_names:
                joint.set("type", "fixed")
                for tag in ("axis", "limit", "dynamics"):
                    for element in joint.findall(tag):
                        joint.remove(element)
        out = String()
        out.data = ET.tostring(root, encoding="unicode")
        self._desc_out.publish(out)

    def _on_raw_states(self, msg):
        """Viewer mode: replace the ram joints with exact values and forward."""
        if not msg.name:
            return  # the slider node sends an empty message before it has a robot
        ram_names = {n for group in RAM_JOINTS.values() for n in group}
        names, positions = [], []
        for n, q in zip(msg.name, msg.position):
            if n not in ram_names:
                names.append(n)
                positions.append(q)
        try:
            q_lift = positions[names.index("bucket_lift_joint")]
            q_tilt = positions[names.index("bucket_tilt_joint")]
            q_jaw = positions[names.index("bucket_jaw_joint")]
        except ValueError:
            self._out.publish(msg)  # no bucket in this description: pass through
            return
        v = ram_values(q_lift, q_tilt, q_jaw)
        for key, joints in (
            ("lift", RAM_JOINTS["lift"]),
            ("tilt", RAM_JOINTS["tilt"]),
            ("jaw", RAM_JOINTS["jaw"]),
        ):
            for i, joint in enumerate(joints):
                names.append(joint)
                positions.append(v[f"{key}_{'angle' if i % 2 == 0 else 'extend'}"])
        out = JointState()
        out.header = msg.header
        out.name = names
        out.position = [float(x) for x in positions]
        self._out.publish(out)

    def _on_states(self, msg):
        # joint_states arrives from two publishers with different joint sets, so
        # take whichever of ours is present and ignore the rest
        try:
            q_lift = msg.position[msg.name.index("bucket_lift_joint")]
            q_tilt = msg.position[msg.name.index("bucket_tilt_joint")]
        except (ValueError, IndexError):
            return
        # the jaw joint is absent from older descriptions; treat it as closed
        try:
            q_jaw = msg.position[msg.name.index("bucket_jaw_joint")]
        except (ValueError, IndexError):
            q_jaw = 0.0

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last is not None and now - self._last < self._min_dt:
            return
        self._last = now

        for key, value in ram_values(q_lift, q_tilt, q_jaw).items():
            self._pub[key].publish(Float64(data=float(value)))


def main():
    rclpy.init()
    node = BucketRamFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
