#!/usr/bin/env python3
"""Teach console for writing bucket behaviours: jog, zero, save poses, run trees.

    ros2 run payloads bucket_teach.py [--dir DIR] [--invert lift,tilt,jaw]

The loop it is for:

  1. z   zero the encoders (bucket_driver's /bucket/zero_encoder)
  2. jog the bucket to a pose with the keys below, then
     p   save it as Pose_<name> in bucket_poses.xml, and
     g   send the bucket back to a saved pose to check it
  3. arrange the Pose_* subtrees into a tree in Groot2 (open bucket.btproj there)
  4. r   run that tree; Groot2's Monitor on port 1667 shows it live. The run waits
         --groot-delay seconds before moving so you can click Connect, and keeps the
         finished tree on Groot2 for --groot-hold seconds (space ends it early)

Jogging goes through bucket_driver exactly as the gamepad does (Actuators on
bucket_actuators), so the first jog takes the bucket from autonomy. Poses and trees
run through `bucket_cli`, which hands the bucket back (/bucket/rearm) first. Moves are
direct: the board drives flat out to each pose, so speed is not a pose setting.

Keys (hold to jog):
  w / s  lift up / down      a / d  tilt curl / dump      q / e  jaw close / open
  + / -  jog duty            space  stop (also stops a running pose or tree)
  z zero   p save pose   l list poses   x delete pose   g go to pose   r run tree
  h hand the bucket back to autonomy   Q quit

DIR defaults to the source copy of autonomy_bringup's behavior_trees/bucket (found
through colcon's symlink-install), so saved poses land in git.
"""

import argparse
import curses
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
from collections import deque

import rclpy
from actuator_msgs.msg import Actuators
from ament_index_python.packages import get_package_share_directory
from interfaces.srv import ZeroBucketEncoder
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

AXES = ("lift", "tilt", "jaw")
JOINTS = {axis: f"bucket_{axis}_joint" for axis in AXES}
# Positive Actuators velocity raises the firmware angle, which is the positive joint
# direction with the xacro's default direction: lift down, tilt dump, jaw open.
JOG_KEYS = {
    "w": ("lift", -1),
    "s": ("lift", +1),
    "a": ("tilt", -1),
    "d": ("tilt", +1),
    "q": ("jaw", -1),
    "e": ("jaw", +1),
}
ZERO_TARGETS = ("lift", "tilt", "jaw")

# Terminals auto-repeat a held key only after ~0.5 s, so the first press jogs that
# long and each repeat extends it a little. Letting go stops it within REPEAT_HOLD_S.
FIRST_HOLD_S = 0.55
REPEAT_HOLD_S = 0.15
# bucket_driver zeros its speeds after 100 ms without a command; publish well inside.
LOOP_S = 0.05
# The actuators don't move below ~10 % duty.
JOG_STEPS = (0.15, 0.25, 0.4, 0.6, 0.8, 1.0)
DEFAULT_MAX_ACTUATOR_SPEED = 0.1
CALL_TIMEOUT_S = 3.0
POSE_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def default_dir() -> str:
    installed = os.path.join(
        get_package_share_directory("autonomy_bringup"), "behavior_trees", "bucket"
    )
    poses = os.path.join(installed, "bucket_poses.xml")
    return (
        os.path.dirname(os.path.realpath(poses)) if os.path.exists(poses) else installed
    )


# --- bucket_poses.xml ------------------------------------------------------------


class PoseFile:
    """Pose_<name> subtrees in bucket_poses.xml, each one direct MoveBucket."""

    def __init__(self, path: str):
        self.path = path

    def _load(self) -> ET.ElementTree:
        if not os.path.exists(self.path):
            return ET.ElementTree(ET.Element("root", {"BTCPP_format": "4"}))
        parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
        return ET.parse(self.path, parser)

    def poses(self) -> dict:
        """{name: {attribute: value}} of each pose's MoveBucket, in file order."""
        result = {}
        for tree in self._load().getroot().iter("BehaviorTree"):
            tree_id = tree.get("ID", "")
            move = tree.find("MoveBucket")
            if tree_id.startswith("Pose_") and move is not None:
                result[tree_id[len("Pose_") :]] = dict(move.attrib)
        return result

    def save(self, name: str, angles: dict, timeout_s: float, attempts: int):
        tree = self._load()
        root = tree.getroot()
        move = ET.Element("MoveBucket")
        for axis, degrees in angles.items():
            move.set(f"{axis}_deg", f"{degrees:.1f}")
        move.set("timeout_s", f"{timeout_s:g}")
        move.set("attempts", str(attempts))
        move.set("server_name", "{@bucket_action}")
        existing = self._find(root, name)
        if existing is not None:
            for child in list(existing):
                existing.remove(child)
            existing.append(move)
        else:
            subtree = ET.SubElement(root, "BehaviorTree", {"ID": f"Pose_{name}"})
            subtree.append(move)
        self._write(tree)

    def delete(self, name: str) -> bool:
        tree = self._load()
        existing = self._find(tree.getroot(), name)
        if existing is None:
            return False
        tree.getroot().remove(existing)
        self._write(tree)
        return True

    @staticmethod
    def _find(root, name):
        return next(
            (t for t in root.iter("BehaviorTree") if t.get("ID") == f"Pose_{name}"),
            None,
        )

    def _write(self, tree: ET.ElementTree):
        ET.indent(tree, space="  ")
        text = ET.tostring(tree.getroot(), encoding="unicode").replace(" />", "/>")
        with open(self.path, "w") as f:
            f.write('<?xml version="1.0" encoding="UTF-8"?>\n' + text + "\n")


def tree_ids(directory: str) -> list:
    """Every BehaviorTree ID in the directory's files except the poses."""
    ids = []
    for name in sorted(os.listdir(directory)):
        if not name.endswith(".xml"):
            continue
        try:
            root = ET.parse(os.path.join(directory, name)).getroot()
        except ET.ParseError:
            continue
        ids += [
            t.get("ID")
            for t in root.iter("BehaviorTree")
            if t.get("ID") and not t.get("ID").startswith("Pose_")
        ]
    return ids


# --- ROS side --------------------------------------------------------------------


class TeachNode(Node):
    def __init__(self):
        super().__init__("bucket_teach")
        self.angles = {}  # axis -> degrees, from /joint_states
        self.override = None  # bucket_driver's latched /bucket/operator_override
        self._actuators = self.create_publisher(Actuators, "bucket_actuators", 10)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            Bool, "/bucket/operator_override", self._on_override, latched
        )
        self._rearm = self.create_client(Trigger, "/bucket/rearm")
        self._zero = self.create_client(ZeroBucketEncoder, "/bucket/zero_encoder")
        self._driver_params = self.create_client(
            GetParameters, "/bucket_driver/get_parameters"
        )

    def _on_joints(self, msg: JointState):
        for axis, joint in JOINTS.items():
            if joint in msg.name:
                self.angles[axis] = math.degrees(msg.position[msg.name.index(joint)])

    def _on_override(self, msg: Bool):
        self.override = msg.data

    def publish_speeds(self, speeds: dict):
        msg = Actuators()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.velocity = [float(speeds.get(axis, 0.0)) for axis in AXES]
        self._actuators.publish(msg)

    def _call(self, client, request):
        if not client.wait_for_service(timeout_sec=1.0):
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + CALL_TIMEOUT_S
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def max_actuator_speed(self):
        request = GetParameters.Request(names=["max_actuator_speed"])
        response = self._call(self._driver_params, request)
        if response is None or not response.values:
            return None
        return response.values[0].double_value or None

    def rearm(self):
        return self._call(self._rearm, Trigger.Request())

    def zero(self, target: str):
        return self._call(self._zero, ZeroBucketEncoder.Request(target=target))


# --- console ---------------------------------------------------------------------


class Console:
    def __init__(self, screen, node: TeachNode, args):
        self.screen = screen
        self.node = node
        self.dir = args.dir
        self.poses = PoseFile(os.path.join(self.dir, "bucket_poses.xml"))
        self.invert = {axis: axis in args.invert for axis in AXES}
        self.groot_port = args.groot_port
        self.groot_delay = args.groot_delay
        self.groot_hold = args.groot_hold
        # Appended to by the thread following a running bucket_cli as well.
        self.log_lines = deque(maxlen=500)
        self.log_lock = threading.Lock()
        self.jog_step = 5
        self.jog_axis = None
        self.jog_sign = 0
        self.jog_until = 0.0
        self.stop_frames = 0
        self.process = None
        self.max_speed = node.max_actuator_speed()
        if self.max_speed is None:
            self.max_speed = DEFAULT_MAX_ACTUATOR_SPEED
            self.log(
                f"bucket_driver did not answer for max_actuator_speed - assuming "
                f"{DEFAULT_MAX_ACTUATOR_SPEED}. Is it running?"
            )
        self.log(f"poses and trees: {self.dir}")

    # -- output

    def log(self, text: str):
        with self.log_lock:
            for line in str(text).splitlines() or [""]:
                self.log_lines.append(line)

    def draw(self, prompt: str = ""):
        screen = self.screen
        screen.erase()
        height, width = screen.getmaxyx()

        def put(y, text, attr=0):
            if 0 <= y < height:
                screen.addnstr(y, 0, text.ljust(width - 1), width - 1, attr)

        angles = "   ".join(
            f"{axis} {self.node.angles[axis]:7.1f}°"
            if axis in self.node.angles
            else f"{axis}     ---"
            for axis in AXES
        )
        owner = {
            True: "operator has the bucket",
            False: "autonomy has the bucket",
            None: "bucket_driver not seen",
        }[self.node.override]
        jog = f"jog {int(JOG_STEPS[self.jog_step] * 100)}%"
        if self.jogging():
            jog += f"  moving {self.jog_axis} {'+' if self.jog_sign > 0 else '-'}"
        running = "   RUNNING (space stops)" if self.process else ""
        put(0, " Perseus bucket teach", curses.A_BOLD)
        put(1, f" {angles}     {owner}", curses.A_REVERSE)
        put(2, f" {jog}{running}")
        put(3, " w/s lift  a/d tilt  q/e jaw  +/- duty  space stop")
        put(
            4,
            " z zero  p save pose  l list  x delete  g go to pose  r run tree  h hand back  Q quit",
        )
        put(5, "─" * (width - 1))
        body = height - 7
        with self.log_lock:
            lines = list(self.log_lines)[-body:] if body > 0 else []
        for i, line in enumerate(lines):
            put(6 + i, " " + line)
        put(height - 1, prompt, curses.A_BOLD)
        screen.refresh()

    # -- jogging

    def jogging(self) -> bool:
        return self.jog_axis is not None and time.monotonic() < self.jog_until

    def jog(self, axis: str, sign: int):
        now = time.monotonic()
        repeat = self.jogging() and axis == self.jog_axis and sign == self.jog_sign
        self.jog_axis, self.jog_sign = axis, sign
        self.jog_until = now + (REPEAT_HOLD_S if repeat else FIRST_HOLD_S)

    def stop(self):
        self.jog_axis = None
        self.stop_frames = 3

    def publish(self):
        if self.jogging():
            sign = -self.jog_sign if self.invert[self.jog_axis] else self.jog_sign
            speed = sign * JOG_STEPS[self.jog_step] * self.max_speed
            self.node.publish_speeds({self.jog_axis: speed})
            self.stop_frames = 3
        elif self.stop_frames > 0:
            # A few zeros, so letting go stops at once rather than on the driver's
            # 100 ms timeout.
            self.jog_axis = None
            self.node.publish_speeds({})
            self.stop_frames -= 1

    def settle(self):
        """Stops jogging and waits until bucket_driver no longer counts a stick held,
        which /bucket/rearm refuses."""
        self.stop()
        for _ in range(4):
            self.publish()
            time.sleep(LOOP_S)
        time.sleep(0.15)

    # -- prompts

    def ask(self, question: str, default: str = "") -> str:
        self.settle()
        suffix = f" [{default}]" if default else ""
        self.draw(f" {question}{suffix}: ")
        curses.echo()
        curses.curs_set(1)
        self.screen.nodelay(False)
        try:
            height, width = self.screen.getmaxyx()
            column = min(len(question) + len(suffix) + 3, width - 2)
            answer = self.screen.getstr(height - 1, column, 80).decode(errors="ignore")
        finally:
            curses.noecho()
            curses.curs_set(0)
            self.screen.nodelay(True)
        return answer.strip() or default

    def ask_number(self, question: str, default, kind=float):
        text = self.ask(question, str(default))
        try:
            value = kind(text)
        except ValueError:
            self.log(f"'{text}' is not a number - using {default}")
            return default
        return value if value > 0 else default

    def pick(self, title: str, options: list):
        if not options:
            return None
        self.log(title)
        for i, option in enumerate(options, 1):
            self.log(f"  {i:2d}  {option}")
        answer = self.ask("number or name (empty to cancel)")
        if not answer:
            return None
        if answer.isdigit() and 1 <= int(answer) <= len(options):
            return options[int(answer) - 1]
        if answer in options:
            return answer
        self.log(f"no '{answer}'")
        return None

    # -- commands

    def zero(self):
        target = self.pick(
            "Zero which bank? (both of its encoders)",
            list(ZERO_TARGETS),
        )
        if target is None:
            return
        axis = target
        if self.ask(f"zero {target} where it stands? type yes") != "yes":
            self.log("not zeroed")
            return
        response = self.node.zero(target)
        if response is None:
            self.log("/bucket/zero_encoder did not answer - is bucket_driver running?")
            return
        self.log(("" if response.success else "refused: ") + response.message)
        if not response.success:
            return
        time.sleep(1.0)
        reading = self.node.angles.get(axis)
        if reading is not None and abs(reading) < 0.5:
            self.log(f"{axis} now reads {reading:.1f}°")
        else:
            shown = "---" if reading is None else f"{reading:.1f}°"
            self.log(
                f"{axis} reads {shown}, not 0: the bucket firmware does not handle "
                "the zero frame yet (or offset_deg is set in the xacro)."
            )

    def save_pose(self):
        missing = [axis for axis in AXES if axis not in self.node.angles]
        if missing:
            self.log(
                f"no reading for {', '.join(missing)} on /joint_states - not saved"
            )
            return
        name = self.ask("pose name (letters, digits, _)")
        if not name:
            return
        if not POSE_RE.match(name):
            self.log(f"'{name}' is not a usable name")
            return
        existing = self.poses.poses()
        if (
            name in existing
            and self.ask(f"Pose_{name} exists - replace it? y/n", "n") != "y"
        ):
            return
        joints = self.ask("joints to include (l t j)", "ltj")
        chosen = [axis for axis in AXES if axis[0] in joints]
        if not chosen:
            self.log("no joints chosen - not saved")
            return
        timeout_s = self.ask_number("timeout_s", 6.0)
        attempts = int(self.ask_number("attempts", 2, int))
        angles = {axis: self.node.angles[axis] for axis in chosen}
        self.poses.save(name, angles, timeout_s, attempts)
        shown = ", ".join(f"{axis} {degrees:.1f}" for axis, degrees in angles.items())
        self.log(
            f"saved Pose_{name}: {shown} deg, timeout {timeout_s:g} s, {attempts} attempt(s)"
        )
        self.refresh_project()

    def list_poses(self):
        poses = self.poses.poses()
        if not poses:
            self.log("no poses saved yet")
            return
        self.log("saved poses:")
        for name, attributes in poses.items():
            shown = "  ".join(
                f"{axis} {attributes[f'{axis}_deg']}"
                for axis in AXES
                if attributes.get(f"{axis}_deg")  # Groot2 saves an omitted joint as ""
            )
            self.log(
                f"  {name:16s} {shown}   timeout {attributes.get('timeout_s', '?')} s, "
                f"{attributes.get('attempts', '1')} attempt(s)"
            )

    def delete_pose(self):
        name = self.pick("Delete which pose?", list(self.poses.poses()))
        if name and self.ask(f"delete Pose_{name}? type yes") == "yes":
            self.poses.delete(name)
            self.log(f"deleted Pose_{name} - any tree using it will no longer load")

    def go_to_pose(self):
        name = self.pick("Go to which pose?", list(self.poses.poses()))
        if name:
            self.run_cli(["pose", name])

    def run_tree(self):
        tree = self.pick(
            f"Run which tree? (Groot2: Monitor -> connect, port {self.groot_port})",
            tree_ids(self.dir),
        )
        if tree:
            self.run_cli(
                [
                    "run",
                    tree,
                    "--start-delay",
                    f"{self.groot_delay:g}",
                    "--hold",
                    f"{self.groot_hold:g}",
                ]
            )

    def hand_back(self):
        self.settle()
        if self.node.override is False:
            self.log("autonomy already has the bucket")
            return
        response = self.node.rearm()
        if response is None:
            self.log("/bucket/rearm did not answer - is bucket_driver running?")
        else:
            self.log(
                ("re-armed: " if response.success else "refused: ") + response.message
            )

    def refresh_project(self):
        result = subprocess.run(
            [
                "ros2",
                "run",
                "mission_bt_server",
                "bucket_cli",
                "project",
                "--dir",
                self.dir,
            ],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            self.log("could not refresh bucket.btproj:")
            self.log(result.stderr or result.stdout)

    def run_cli(self, arguments: list):
        if self.process:
            self.log("something is already running - space stops it")
            return
        self.settle()
        command = [
            "ros2",
            "run",
            "mission_bt_server",
            "bucket_cli",
            *arguments,
            "--dir",
            self.dir,
            "--rearm",
            "--groot-port",
            str(self.groot_port),
        ]
        self.log("$ " + " ".join(command[3:]))
        # Its own process group, so space interrupts bucket_cli (which halts the tree
        # and cancels the goal) and not this console.
        self.process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        threading.Thread(target=self._follow, args=(self.process,), daemon=True).start()

    def _follow(self, process):
        for line in process.stdout:
            self.log("  " + line.rstrip())
        code = process.wait()
        self.log("  -> done" if code == 0 else f"  -> exited {code}")
        self.process = None

    def interrupt(self):
        if self.process:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            self.log("  stopping - the bucket holds where it is")

    # -- main loop

    def run(self):
        curses.curs_set(0)
        self.screen.nodelay(True)
        commands = {
            "z": self.zero,
            "p": self.save_pose,
            "l": self.list_poses,
            "x": self.delete_pose,
            "g": self.go_to_pose,
            "r": self.run_tree,
            "h": self.hand_back,
        }
        while rclpy.ok():
            started = time.monotonic()
            key = None
            while (code := self.screen.getch()) != -1:
                key = chr(code) if 0 <= code < 256 else None
                if key in JOG_KEYS:
                    self.jog(*JOG_KEYS[key])
                elif key == " ":
                    self.stop()
                    self.interrupt()
                elif key in ("+", "="):
                    self.jog_step = min(self.jog_step + 1, len(JOG_STEPS) - 1)
                elif key in ("-", "_"):
                    self.jog_step = max(self.jog_step - 1, 0)
                elif key == "Q":
                    self.settle()
                    self.interrupt()
                    return
                elif key in commands:
                    commands[key]()
            self.publish()
            self.draw()
            time.sleep(max(0.0, LOOP_S - (time.monotonic() - started)))


def main():
    rclpy.init(args=sys.argv)
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dir", default=None, help="bucket behaviours directory")
    parser.add_argument(
        "--invert",
        default="",
        help="comma-separated axes (lift,tilt,jaw) whose jog keys move the wrong way",
    )
    parser.add_argument("--groot-port", type=int, default=1667)
    parser.add_argument(
        "--groot-delay",
        type=float,
        default=3.0,
        help="seconds a tree run (r) waits before moving, to connect Groot2",
    )
    parser.add_argument(
        "--groot-hold",
        type=float,
        default=30.0,
        help="seconds a finished tree stays on Groot2 (space ends it early)",
    )
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    args.dir = os.path.abspath(args.dir) if args.dir else default_dir()
    args.invert = {axis.strip() for axis in args.invert.split(",") if axis.strip()}
    if not os.path.isdir(args.dir):
        print(
            f"no bucket behaviours directory at {args.dir} - pass --dir",
            file=sys.stderr,
        )
        sys.exit(1)

    node = TeachNode()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        curses.wrapper(lambda screen: Console(screen, node, args).run())
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_speeds({})
        print(
            "bucket_teach: jog stopped. If you jogged, the operator still has the bucket - "
            "`ros2 run mission_bt_server bucket_cli rearm` hands it back to autonomy."
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
