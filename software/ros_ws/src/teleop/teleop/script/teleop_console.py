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
  m  manual: the gamepad is live again (the deadman still has to be held).
  Q  quit.

While armed, holding the drive deadman takes control straight back, without this
console. The bucket is on the same deadman: it moves only while the deadman is held, and
it is taken from autonomy only once a bucket stick also moves (bucket_driver's
override_deadband).
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
# The launch's nodes colour their logs (emulate_tty); curses would show the codes.
ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")


class ConsoleNode(Node):
    def __init__(self):
        super().__init__("teleop_console")
        self.armed = None  # generic_controller's latched /teleop/autonomy_armed
        self.override = None  # bucket_driver's latched /bucket/operator_override
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

    def _on_armed(self, msg: Bool):
        self.armed = msg.data

    def _on_override(self, msg: Bool):
        self.override = msg.data

    def _on_joints(self, msg: JointState):
        for axis, joint in JOINTS.items():
            if joint in msg.name:
                self.angles[axis] = math.degrees(msg.position[msg.name.index(joint)])

    def _on_joy(self, _msg: Joy):
        self.joy_time = time.monotonic()

    def _on_joy_vel(self, _msg: TwistStamped):
        self.joy_vel_time = time.monotonic()

    def controller_running(self) -> bool:
        return any(
            name == CONTROLLER_NODE for name, _ in self.get_node_names_and_namespaces()
        )

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

        if node.controller_running():
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
            False: "MANUAL - gamepad live while the deadman is held",
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
        drive = "joystick driving" if driving else "navigation has the drive"
        live = self.node.joy_time is not None and now - self.node.joy_time < JOY_STALE_S
        gamepad = "gamepad live" if live else "gamepad NOT SEEN"
        angles = "   ".join(
            f"{axis} {self.node.angles[axis]:7.1f}°"
            if axis in self.node.angles
            else f"{axis}     ---"
            for axis in AXES
        )
        launched = "   (controller started here)" if self.process else ""

        put(0, " Perseus teleop console", curses.A_BOLD)
        put(1, f" {mode}", curses.A_REVERSE)
        put(2, f" {drive}   |   {bucket}   |   {gamepad}")
        put(3, f" {angles}{launched}")
        put(4, " a arm autonomy   m manual   Q quit")
        put(5, "─" * (width - 1))
        body = height - 6
        with self.log_lock:
            lines = list(self.log_lines)[-body:] if body > 0 else []
        for i, line in enumerate(lines):
            put(6 + i, " " + line)
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
    # Let discovery and the latched topics arrive before deciding whether to launch.
    time.sleep(1.0)
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
            True: " The operator still has the bucket - "
            "`ros2 run mission_bt_server bucket_cli rearm` hands it back.",
            False: "",
            None: "",
        }[node.override]
        print(f"teleop_console: {owner}.{bucket}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
