#!/usr/bin/env python3
"""Operator console: drive with the gamepad, hand the rover back to autonomy.

    ros2 run teleop teleop_console [--type taranis] [--wired] [--dual-stick]
                                   [--config PATH] [--no-launch]

One program for the controller side. It starts joy_node and generic_controller
(teleop's controller.launch.py) unless a generic_controller is already running - from
base_station.launch.py, say - in which case it attaches to that one.

  a  arm autonomy: generic_controller stops the drive and the bucket, hands the bucket
     back to its trajectory controller (/bucket/rearm), then publishes nothing at all,
     so the gamepad cannot fight navigation or the bucket's autonomy.
  m  manual: the rover stops and the gamepad is live again (the deadman still has to be
     held).
  Q  quit.

Autonomy can only move the rover while armed. In manual, generic_controller holds
twist_mux's /teleop/navigation_lock, which masks navigation, and takes the bucket from
its trajectory controller (/bucket/take_control); mission_bt_server and bucket_cli
refuse to start a tree and stop the one running. So only the operator moves anything
until `a` is pressed again.

While armed, holding the drive deadman takes control straight back, without this
console. The bucket is on the same deadman: it moves only while the deadman is held.
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
from collections import deque

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

AXES = ("lift", "tilt", "jaw")
JOINTS = {axis: f"bucket_{axis}_joint" for axis in AXES}
CONTROLLER_NODE = "generic_controller"
CONTROLLER_TYPES = ("taranis", "xbox", "logitech", "8bitdo")

LOOP_S = 0.05
CALL_TIMEOUT_S = 8.0  # arming waits up to 5 s on /bucket/rearm
# twist_mux's joy_vel timeout: quieter than this and navigation has the drive.
JOY_VEL_TIMEOUT_S = 0.5
# Longer than generic_controller's 500 ms timeout_ns, so this only says NOT SEEN
# once the controller has already stopped the rover.
JOY_STALE_S = 1.0
# How long manual may leave autonomy with the bucket before it counts as unlocked:
# generic_controller asks at once, and the takeover settles within 200 ms.
BUCKET_TAKE_GRACE_S = 1.5
# How long to look for a running generic_controller before starting one. Discovery
# across machines can take seconds, and starting a second controller by mistake
# leaves one in manual taking the bucket back from autonomy.
DISCOVERY_S = 3.0
# The launch's nodes colour their logs (emulate_tty); curses would show the codes.
ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")


class ConsoleNode(Node):
    def __init__(self):
        super().__init__("teleop_console")
        self.armed = None  # generic_controller's latched /teleop/autonomy_armed
        self.override = None  # bucket_driver's latched /bucket/operator_override
        # Since when manual has left autonomy with the bucket, if it has.
        self.unlocked_since = None
        self.angles = {}  # axis -> degrees, from /joint_states
        self.joy_time = None
        self.joy_vel_time = None
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            Bool, "/teleop/autonomy_armed", self._on_armed, latched
        )
        self.create_subscription(
            Bool, "/bucket/operator_override", self._on_override, latched
        )
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(Joy, "/joy", self._on_joy, 10)
        self.create_subscription(TwistStamped, "/joy_vel", self._on_joy_vel, 10)
        self._arm = self.create_client(Trigger, "/teleop/arm_autonomy")
        self._take_control = self.create_client(Trigger, "/teleop/take_control")
        # Only to see whether bucket_driver serves it; generic_controller calls it.
        self._bucket_take = self.create_client(Trigger, "/bucket/take_control")

    def _on_armed(self, msg: Bool):
        self.armed = msg.data
        self._track_unlocked()

    def _on_override(self, msg: Bool):
        self.override = msg.data
        self._track_unlocked()

    def _track_unlocked(self):
        if self.armed is False and self.override is False:
            if self.unlocked_since is None:
                self.unlocked_since = time.monotonic()
        else:
            self.unlocked_since = None

    def lockout_problems(self) -> list:
        """What manual is failing to lock out, empty while it holds (or when armed)."""
        problems = []
        if self.count_publishers("/teleop/autonomy_armed") > 1:
            problems.append(
                "TWO generic_controllers running: the one in manual takes the bucket "
                "from autonomy and locks navigation out - stop one of them"
            )
        if self.armed is not False:
            return problems
        if self.count_subscribers("/teleop/navigation_lock") == 0:
            problems.append(
                "DRIVE NOT LOCKED: twist_mux is not reading /teleop/navigation_lock - "
                "deploy perseus' twist_mux.yaml to the rover and restart twist_mux"
            )
        if (
            self.unlocked_since is not None
            and time.monotonic() - self.unlocked_since > BUCKET_TAKE_GRACE_S
        ):
            problems.append(
                "BUCKET NOT LOCKED: autonomy still has the bucket - "
                + (
                    "generic_controller cannot take it, see its log"
                    if self._bucket_take.service_is_ready()
                    else "bucket_driver has no /bucket/take_control: rebuild payloads "
                    "on the rover and restart bucket_driver"
                )
            )
        return problems

    def _on_joints(self, msg: JointState):
        for axis, joint in JOINTS.items():
            if joint in msg.name:
                self.angles[axis] = math.degrees(msg.position[msg.name.index(joint)])

    def _on_joy(self, _msg: Joy):
        self.joy_time = time.monotonic()

    def _on_joy_vel(self, _msg: TwistStamped):
        self.joy_vel_time = time.monotonic()

    def controller_running(self) -> bool:
        return self.count_publishers("/teleop/autonomy_armed") > 0 or any(
            name == CONTROLLER_NODE for name, _ in self.get_node_names_and_namespaces()
        )

    def wait_for_controller(self) -> bool:
        """Whether a generic_controller turns up within DISCOVERY_S."""
        deadline = time.monotonic() + DISCOVERY_S
        while time.monotonic() < deadline:
            if self.controller_running():
                return True
            time.sleep(0.1)
        return self.controller_running()

    def _call(self, client, request):
        if not client.wait_for_service(timeout_sec=1.0):
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + CALL_TIMEOUT_S
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def arm(self):
        return self._call(self._arm, Trigger.Request())

    def take_control(self):
        return self._call(self._take_control, Trigger.Request())


class Console:
    def __init__(self, screen, node: ConsoleNode, args):
        self.screen = screen
        self.node = node
        # Appended to by the thread following the controller launch as well.
        self.log_lines = deque(maxlen=500)
        self.log_lock = threading.Lock()
        self.process = None
        self.busy = ""
        self.arm_thread = None
        self.reported = set()  # lockout problems already logged

        if node.wait_for_controller():
            self.log("attached to the running generic_controller")
        elif args.no_launch:
            self.log("no generic_controller running - start one, or drop --no-launch")
        else:
            self.launch_controller(args)

    # -- output

    def log(self, text: str):
        with self.log_lock:
            for line in str(text).splitlines() or [""]:
                self.log_lines.append(line)

    def draw(self):
        screen = self.screen
        screen.erase()
        height, width = screen.getmaxyx()

        def put(y, text, attr=0):
            if 0 <= y < height:
                screen.addnstr(y, 0, text.ljust(width - 1), width - 1, attr)

        now = time.monotonic()
        mode = {
            True: "AUTONOMY ARMED - gamepad silent, deadman takes over",
            False: "MANUAL - autonomy locked out, gamepad live while the deadman is held",
            None: "generic_controller not seen",
        }[self.node.armed]
        if self.busy:
            mode = self.busy
        bucket = {
            True: "operator has the bucket",
            False: "autonomy has the bucket",
            None: "bucket_driver not seen",
        }[self.node.override]
        driving = (
            self.node.joy_vel_time is not None
            and now - self.node.joy_vel_time < JOY_VEL_TIMEOUT_S
        )
        if driving:
            drive = "joystick driving"
        elif self.node.armed:
            drive = "navigation has the drive"
        elif self.node.count_subscribers("/teleop/navigation_lock"):
            drive = "navigation locked out"
        else:
            drive = "navigation NOT locked"
        live = self.node.joy_time is not None and now - self.node.joy_time < JOY_STALE_S
        gamepad = "gamepad live" if live else "gamepad NOT SEEN"
        angles = "   ".join(
            f"{axis} {self.node.angles[axis]:7.1f}°"
            if axis in self.node.angles
            else f"{axis}     ---"
            for axis in AXES
        )
        launched = "   (controller started here)" if self.process else ""

        problems = self.node.lockout_problems()
        for problem in set(problems) - self.reported:
            self.log("!! " + problem)
        self.reported = set(problems)

        put(0, " Perseus teleop console", curses.A_BOLD)
        put(1, f" {mode}", curses.A_REVERSE)
        row = 2
        for problem in problems:
            put(row, f" !! {problem}", curses.A_BOLD | curses.A_REVERSE)
            row += 1
        put(row, f" {drive}   |   {bucket}   |   {gamepad}")
        put(row + 1, f" {angles}{launched}")
        put(row + 2, " a arm autonomy   m manual   Q quit")
        put(row + 3, "─" * (width - 1))
        top = row + 4
        body = height - top
        with self.log_lock:
            lines = list(self.log_lines)[-body:] if body > 0 else []
        for i, line in enumerate(lines):
            put(top + i, " " + line)
        screen.refresh()

    # -- the controller

    def launch_controller(self, args):
        command = ["ros2", "launch", "teleop", "controller.launch.py", "debug:=false"]
        if args.config:
            command.append(f"config:={os.path.abspath(args.config)}")
        else:
            command += [
                f"type:={args.type}",
                f"wireless:={'false' if args.wired else 'true'}",
                f"dual_stick:={'true' if args.dual_stick else 'false'}",
            ]
        self.log("$ " + " ".join(command))
        # Its own process group, so quitting interrupts the whole launch.
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
            self.log("  " + ANSI_RE.sub("", line.rstrip()))
        code = process.wait()
        self.log(f"  -> controller exited {code}")
        self.process = None

    def stop_controller(self):
        process = self.process
        if not process:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5.0)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)

    # -- commands

    def arm(self):
        # Off the main loop: arming can wait seconds on /bucket/rearm.
        if self.arm_thread and self.arm_thread.is_alive():
            self.log("already arming")
            return
        self.busy = "ARMING..."
        self.arm_thread = threading.Thread(target=self._arm, daemon=True)
        self.arm_thread.start()

    def _arm(self):
        try:
            response = self.node.arm()
            if response is None:
                self.log(
                    "/teleop/arm_autonomy did not answer - is generic_controller running?"
                )
            else:
                self.log(
                    ("armed: " if response.success else "refused: ") + response.message
                )
        finally:
            self.busy = ""

    def take_control(self):
        response = self.node.take_control()
        if response is None:
            self.log(
                "/teleop/take_control did not answer - is generic_controller running?"
            )
        else:
            self.log(response.message)

    # -- main loop

    def run(self):
        # curses.wrapper starts colour, which assumes white on black; keep the
        # terminal's own foreground and background instead.
        try:
            curses.use_default_colors()
        except curses.error:
            pass
        curses.curs_set(0)
        self.screen.nodelay(True)
        commands = {"a": self.arm, "m": self.take_control}
        while rclpy.ok():
            started = time.monotonic()
            while (code := self.screen.getch()) != -1:
                key = chr(code) if 0 <= code < 256 else None
                if key == "Q":
                    self.stop_controller()
                    return
                if key in commands:
                    commands[key]()
            self.draw()
            time.sleep(max(0.0, LOOP_S - (time.monotonic() - started)))


def main():
    rclpy.init(args=sys.argv)
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--type",
        default="taranis",
        choices=CONTROLLER_TYPES,
        help="controller config, as controller.launch.py type:=",
    )
    parser.add_argument(
        "--wired", action="store_true", help="xbox: wired, not wireless"
    )
    parser.add_argument(
        "--dual-stick", action="store_true", help="xbox: dual-stick driving"
    )
    parser.add_argument(
        "--config", default="", help="controller config file, overrides --type"
    )
    parser.add_argument(
        "--no-launch",
        action="store_true",
        help="never start a controller, only attach to a running one",
    )
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    node = ConsoleNode()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    # Let the latched topics arrive; Console waits on discovery before launching.
    time.sleep(0.5)
    console = None
    try:

        def start(screen):
            nonlocal console
            console = Console(screen, node, args)
            console.run()

        curses.wrapper(start)
    except KeyboardInterrupt:
        if console:
            console.stop_controller()
    finally:
        owner = {
            True: "autonomy was armed",
            False: "the gamepad had control",
            None: "generic_controller was never seen",
        }[node.armed]
        bucket = {
            True: " The operator still has the bucket - `a` in teleop_console hands it "
            "back (or `ros2 run mission_bt_server bucket_cli rearm` with no "
            "generic_controller running).",
            False: "",
            None: "",
        }[node.override]
        print(f"teleop_console: {owner}.{bucket}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
