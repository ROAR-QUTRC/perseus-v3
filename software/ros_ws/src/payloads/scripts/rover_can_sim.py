#!/usr/bin/env python3
"""Simulates the rover's CAN bus on a vcan interface: four drive VESCs and the bucket board.

Run perseus.launch.py (or payloads' bucket.launch.py) on the same interface and the
real hardware plugins (VescSystemHardware, BucketHardware) talk to this instead of
the rover, so the whole ros2_control stack, the bucket's behaviours and RViz can be
exercised on a laptop:

    software/scripts/vcan-setup.sh vcan0
    ros2 run payloads rover_can_sim.py --iface vcan0
    ros2 launch perseus perseus.launch.py payload:=bucket can_bus:=vcan0

The bucket board follows firmware/excavation-bucket on this branch, including its
quirks, so what works here should work on the bucket:
  * broadcasts GET_ANGLE for all six encoders and GET_POSITION for the three banks
    every 50 ms, GET_CURRENT every 500 ms (little-endian int16 tenths of a degree,
    uint16 mA);
  * SET_SPEED (int16 duty, +/-32767 = 100 %) puts a bank in velocity mode, except
    that speeds inside +/-1 % don't leave position mode; it stops a bank ~600 ms
    after the last one (200 ms x hi-can's 3 misses);
  * SET_POSITION drives flat out toward the target, stops inside 2 deg and restarts
    past 3 deg, and is never timed out - a bank keeps going to its last target;
  * the actuators don't move below ~10 % duty.

Modes:
  follow (default)   the bucket moves only as commanded
  sweep <axis>       that axis's encoders sweep between --sweep-min and --sweep-max
                     whatever is commanded, to see the model follow the encoder
                     stream on its own

Faults, from the command line (NAME@SECONDS) or typed while it runs:
  drop <encoder>     stop sending one encoder (lift_left, lift_right, tilt_left, ...)
  restore            send every encoder again
  fault <axis>       report a bank fault (GET_FAULT) for lift, tilt or jaws
  clear              clear every bank fault
  bucket off|on      the whole bucket board goes silent / comes back
  sweep <axis>       switch to sweep mode on that axis
  follow             back to follow mode
  status             print the state now
  quit
"""

import argparse
import math
import select
import socket
import struct
import sys
import time
from dataclasses import dataclass, field

# --- SocketCAN -------------------------------------------------------------------

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_FRAME = struct.Struct("=IB3x8s")


class CanBus:
    def __init__(self, iface: str):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((iface,))
        self.sock.setblocking(False)

    def send(self, address: int, data: bytes = b""):
        frame = CAN_FRAME.pack(address | CAN_EFF_FLAG, len(data), data.ljust(8, b"\0"))
        try:
            self.sock.send(frame)
        except OSError:
            pass  # vcan buffer full or interface down: drop, like a busy bus

    def receive(self):
        """Every pending (address, data) data frame, extended IDs only."""
        frames = []
        while True:
            try:
                raw = self.sock.recv(CAN_FRAME.size)
            except BlockingIOError:
                return frames
            can_id, length, data = CAN_FRAME.unpack(raw)
            if can_id & (CAN_ERR_FLAG | CAN_RTR_FLAG) or not can_id & CAN_EFF_FLAG:
                continue
            frames.append((can_id & CAN_EFF_MASK, data[:length]))


# --- hi-can addressing (software/shared/hi-can/include/hi_can_address.hpp) ------


def hican_address(system, subsystem, device, group, parameter):
    return (
        (system << 23) | (subsystem << 20) | (device << 16) | (group << 8) | parameter
    )


BUCKET = (0x04, 0x00, 0x00)  # excavation / bucket / controller
GET_FAULT, GET_CURRENT, SET_SPEED, GET_POSITION, SET_POSITION = (
    0x00,
    0x01,
    0x02,
    0x03,
    0x04,
)
GET_ANGLE = 0x00

FULL_DUTY = 32767
DEADBAND_DUTY = int(1.0 * FULL_DUTY / 100)  # firmware kSpeedDeadband, 1 %
STALL_DUTY = int(10.0 * FULL_DUTY / 100)  # the actuators don't move below ~10 %
HOLD_WINDOW_DEG = 2.0
RESUME_WINDOW_DEG = 3.0
SPEED_TIMEOUT_S = 0.6

# VESC command IDs (hi_can_address.hpp, drive::vesc::command_id)
VESC_SET_RPM, VESC_STATUS_1, VESC_STATUS_4, VESC_STATUS_5 = 3, 9, 16, 27


def vesc_address(vesc, command):
    return (command << 8) | vesc


# --- bucket board ----------------------------------------------------------------


@dataclass
class Bank:
    name: str
    group: int
    encoder_groups: tuple  # (left, right)
    lower: float  # mechanical end stops, joint frame, degrees
    upper: float
    rate: float  # deg/s at full duty
    offset: float = 0.0  # firmware = offset + direction * joint
    direction: float = 1.0
    q: float = 0.0  # joint angle, degrees
    mode: str = "velocity"
    speed: int = 0
    target: float = 0.0  # firmware frame, degrees
    settled: bool = True
    last_speed: float = 0.0
    fault: bool = False
    fault_reported: bool = False
    dropped: set = field(default_factory=set)
    moving: float = 0.0  # last duty actually applied, for status and current

    def firmware_deg(self) -> float:
        return (self.offset + self.direction * self.q) % 360.0

    def set_speed(self, duty: int, now: float):
        self.speed = duty
        self.last_speed = now
        if abs(duty) > DEADBAND_DUTY:
            self.mode = "velocity"

    def set_position(self, degrees: float):
        if self.mode != "position" or abs(degrees - self.target) > 1e-6:
            self.settled = False
        self.target = degrees
        self.mode = "position"

    def step(self, dt: float, now: float):
        if self.mode == "velocity" and now - self.last_speed > SPEED_TIMEOUT_S:
            self.speed = 0
        if self.mode == "position":
            # Shortest way round in the firmware frame, as motor_bank.cpp does.
            error = (self.target - self.firmware_deg() + 180.0) % 360.0 - 180.0
            if self.settled and abs(error) <= RESUME_WINDOW_DEG:
                duty = 0
            elif abs(error) <= HOLD_WINDOW_DEG:
                self.settled = True
                duty = 0
            else:
                self.settled = False
                duty = FULL_DUTY if error > 0 else -FULL_DUTY
        else:
            duty = self.speed
        if abs(duty) < STALL_DUTY:
            duty = 0
        self.moving = duty
        # Positive duty raises the firmware angle (its drive direction is right);
        # direction maps that onto the joint.
        self.q += self.direction * self.rate * (duty / FULL_DUTY) * dt
        self.q = min(max(self.q, self.lower), self.upper)

    def current_ma(self) -> int:
        return 1500 if self.moving else 40


ENCODER_NAMES = {
    f"{bank}_{side}": (bank, i)
    for bank in ("lift", "tilt", "jaws")
    for i, side in enumerate(("left", "right"))
}


def le_int16(value: float) -> bytes:
    return struct.pack("<h", int(round(value)))


class BucketBoard:
    def __init__(self, args):
        self.banks = {
            "lift": Bank("lift", 0x00, (0x03, 0x04), 0.0, 71.4, args.lift_rate),
            "tilt": Bank("tilt", 0x01, (0x05, 0x06), -91.7, 28.1, args.tilt_rate),
            "jaws": Bank("jaws", 0x02, (0x07, 0x08), 0.0, 48.0, args.jaw_rate),
        }
        for name, bank in self.banks.items():
            bank.offset = getattr(args, f"{name}_offset")
            bank.direction = getattr(args, f"{name}_direction")
        self.banks["lift"].q = args.lift_start
        self.banks["tilt"].q = args.tilt_start
        self.online = True
        self.sweep = None  # (bank name, min, max, period)
        self.sweep_t0 = 0.0
        self.last_fast = 0.0
        self.last_slow = 0.0

    def handle(self, address: int, data: bytes, now: float):
        if not self.online:
            return
        prefix = hican_address(*BUCKET, 0, 0)
        if address & ~0xFFFF != prefix:
            return
        group, parameter = (address >> 8) & 0xFF, address & 0xFF
        bank = next((b for b in self.banks.values() if b.group == group), None)
        if bank is None or len(data) != 2:
            return
        value = struct.unpack("<h", data)[0]
        if parameter == SET_SPEED:
            bank.set_speed(value, now)
        elif parameter == SET_POSITION:
            bank.set_position(value / 10.0)

    def step(self, dt: float, now: float):
        for bank in self.banks.values():
            bank.step(dt, now)
        if self.sweep:
            name, low, high, period = self.sweep
            phase = 2 * math.pi * (now - self.sweep_t0) / period
            self.banks[name].q = low + (high - low) * (1 - math.cos(phase)) / 2

    def transmit(self, bus: CanBus, now: float):
        if not self.online:
            return
        if now - self.last_fast >= 0.05:
            self.last_fast = now
            for bank in self.banks.values():
                degrees = bank.firmware_deg() * 10.0
                for side, group in enumerate(bank.encoder_groups):
                    if side not in bank.dropped:
                        bus.send(
                            hican_address(*BUCKET, group, GET_ANGLE), le_int16(degrees)
                        )
                bus.send(
                    hican_address(*BUCKET, bank.group, GET_POSITION), le_int16(degrees)
                )
                if bank.fault or bank.fault_reported:
                    bus.send(
                        hican_address(*BUCKET, bank.group, GET_FAULT),
                        bytes([1 if bank.fault else 0]),
                    )
                    bank.fault_reported = bank.fault
        if now - self.last_slow >= 0.5:
            self.last_slow = now
            for bank in self.banks.values():
                bus.send(
                    hican_address(*BUCKET, bank.group, GET_CURRENT),
                    struct.pack("<H", bank.current_ma()),
                )

    def status(self) -> str:
        parts = []
        for bank in self.banks.values():
            state = (
                f"pos->{bank.target:6.1f}{'*' if bank.settled else ' '}"
                if bank.mode == "position"
                else f"vel {100 * bank.speed / FULL_DUTY:+5.1f}%"
            )
            flags = "".join(
                [" FAULT" if bank.fault else ""]
                + [f" no-{('left', 'right')[s]}" for s in sorted(bank.dropped)]
            )
            parts.append(f"{bank.name} {bank.q:6.1f}deg {state}{flags}")
        if not self.online:
            parts.append("BUCKET OFF")
        if self.sweep:
            parts.append(f"sweeping {self.sweep[0]}")
        return " | ".join(parts)


# --- drive VESCs -----------------------------------------------------------------


@dataclass
class Vesc:
    id: int
    command_erpm: float = 0.0
    erpm: float = 0.0
    last_command: float = 0.0
    revolutions: float = 0.0  # electrical revolutions, the tachometer / 6


class Drive:
    TIME_CONSTANT_S = 0.2
    COMMAND_TIMEOUT_S = 1.0

    def __init__(self):
        self.vescs = [Vesc(i) for i in range(4)]
        self.last_transmit = 0.0

    def handle(self, address: int, data: bytes, now: float):
        command, vesc = (address >> 8) & 0xFF, address & 0xFF
        if address > 0xFFFF or command != VESC_SET_RPM or vesc >= 4 or len(data) != 4:
            return
        self.vescs[vesc].command_erpm = struct.unpack(">i", data)[0]
        self.vescs[vesc].last_command = now

    def step(self, dt: float, now: float):
        for v in self.vescs:
            target = (
                v.command_erpm if now - v.last_command < self.COMMAND_TIMEOUT_S else 0.0
            )
            v.erpm += (target - v.erpm) * min(1.0, dt / self.TIME_CONSTANT_S)
            v.revolutions += v.erpm / 60.0 * dt

    def transmit(self, bus: CanBus, now: float):
        if now - self.last_transmit < 0.02:
            return
        self.last_transmit = now
        for v in self.vescs:
            current = min(abs(v.erpm) / 2000.0, 20.0)
            duty = max(-0.95, min(0.95, v.erpm / 40000.0))
            bus.send(
                vesc_address(v.id, VESC_STATUS_1),
                struct.pack(">ihh", int(v.erpm), int(current * 10), int(duty * 1000)),
            )
            bus.send(
                vesc_address(v.id, VESC_STATUS_4),
                struct.pack(">hhhh", 350, 400, int(current * 10), 0),
            )
            bus.send(
                vesc_address(v.id, VESC_STATUS_5),
                struct.pack(">ihH", int(round(v.revolutions * 6)), 480, 0),
            )

    def status(self) -> str:
        return "erpm " + " ".join(f"{v.erpm:+6.0f}" for v in self.vescs)


# --- control -------------------------------------------------------------------


def apply(command: str, board: BucketBoard, args, now: float) -> bool:
    """Runs one fault/mode command. Returns False to quit."""
    words = command.split()
    if not words:
        return True
    verb, rest = words[0], words[1:]
    if verb == "quit":
        return False
    if verb == "drop" and rest and rest[0] in ENCODER_NAMES:
        bank, side = ENCODER_NAMES[rest[0]]
        board.banks[bank].dropped.add(side)
    elif verb == "restore":
        for bank in board.banks.values():
            bank.dropped.clear()
    elif verb == "fault" and rest and rest[0] in board.banks:
        board.banks[rest[0]].fault = True
    elif verb == "clear":
        for bank in board.banks.values():
            bank.fault = False
    elif verb == "bucket" and rest and rest[0] in ("on", "off"):
        board.online = rest[0] == "on"
    elif verb == "sweep" and rest and rest[0] in board.banks:
        bank = board.banks[rest[0]]
        low = (
            max(args.sweep_min, bank.lower)
            if args.sweep_min is not None
            else bank.lower
        )
        high = (
            min(args.sweep_max, bank.upper)
            if args.sweep_max is not None
            else bank.upper
        )
        board.sweep = (rest[0], low, high, args.sweep_period)
        # Start the cosine where the axis already is, so it doesn't jump.
        fraction = (
            min(max((bank.q - low) / (high - low), 0.0), 1.0) if high > low else 0.0
        )
        board.sweep_t0 = now - math.acos(1 - 2 * fraction) * args.sweep_period / (
            2 * math.pi
        )
    elif verb == "follow":
        board.sweep = None
    elif verb == "status":
        pass
    else:
        print(f"? {command}  (see --help for commands)", flush=True)
        return True
    print(f"> {command}", flush=True)
    return True


def parse_timed(values):
    """['drop lift_left@15', ...] -> [(15.0, 'drop lift_left'), ...]"""
    timed = []
    for value in values or []:
        command, _, at = value.rpartition("@")
        if not command:
            raise SystemExit(f"expected COMMAND@SECONDS, got '{value}'")
        timed.append((float(at), command.strip()))
    return sorted(timed)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--iface", default="vcan0")
    parser.add_argument("--mode", choices=("follow", "sweep"), default="follow")
    parser.add_argument(
        "--axis",
        choices=("lift", "tilt", "jaws"),
        default="lift",
        help="the axis --mode sweep moves",
    )
    parser.add_argument(
        "--sweep-min", type=float, help="degrees, joint frame (default: end stop)"
    )
    parser.add_argument(
        "--sweep-max", type=float, help="degrees, joint frame (default: end stop)"
    )
    parser.add_argument(
        "--sweep-period", type=float, default=10.0, help="seconds per up-and-down"
    )
    parser.add_argument(
        "--lift-rate", type=float, default=5.0, help="deg/s at full duty"
    )
    parser.add_argument(
        "--tilt-rate", type=float, default=8.0, help="deg/s at full duty"
    )
    parser.add_argument(
        "--jaw-rate", type=float, default=10.0, help="deg/s at full duty"
    )
    parser.add_argument(
        "--lift-start", type=float, default=10.0, help="degrees, joint frame"
    )
    parser.add_argument(
        "--tilt-start", type=float, default=0.0, help="degrees, joint frame"
    )
    for axis in ("lift", "tilt", "jaws"):
        parser.add_argument(
            f"--{axis}-offset",
            type=float,
            default=0.0,
            help=f"{axis} firmware zero, degrees: firmware = offset + direction * joint",
        )
        parser.add_argument(
            f"--{axis}-direction", type=float, choices=(1.0, -1.0), default=1.0
        )
    parser.add_argument(
        "--no-bucket", action="store_true", help="start with the bucket board silent"
    )
    parser.add_argument(
        "--no-drive", action="store_true", help="don't simulate the VESCs"
    )
    parser.add_argument(
        "--at",
        action="append",
        metavar="COMMAND@SECONDS",
        help="run a command at a time, e.g. --at 'drop lift_left@15' "
        "--at 'bucket on@30' (repeatable)",
    )
    parser.add_argument(
        "--status-period", type=float, default=1.0, help="seconds; 0 for none"
    )
    args = parser.parse_args()

    try:
        bus = CanBus(args.iface)
    except OSError as e:
        sys.exit(
            f"Can't open {args.iface}: {e}. Set it up with software/scripts/vcan-setup.sh"
        )

    board = BucketBoard(args)
    board.online = not args.no_bucket
    drive = None if args.no_drive else Drive()
    timed = parse_timed(args.at)

    start = time.monotonic()
    if args.mode == "sweep":
        apply(f"sweep {args.axis}", board, args, start)
    print(
        f"Simulating on {args.iface}. Type a command (see --help), Ctrl-C to quit.",
        flush=True,
    )

    interactive = True  # until stdin closes (e.g. /dev/null under ros2 launch)
    last = start
    last_status = start
    try:
        while True:
            readable, _, _ = select.select(
                [bus.sock] + ([sys.stdin] if interactive else []), [], [], 0.005
            )
            now = time.monotonic()
            for address, data in bus.receive():
                board.handle(address, data, now)
                if drive:
                    drive.handle(address, data, now)
            if sys.stdin in readable:
                line = sys.stdin.readline()
                if not line:
                    interactive = False  # stdin closed: keep running headless
                elif not apply(line.strip(), board, args, now):
                    break
            while timed and now - start >= timed[0][0]:
                apply(timed.pop(0)[1], board, args, now)

            board.step(now - last, now)
            if drive:
                drive.step(now - last, now)
            last = now
            board.transmit(bus, now)
            if drive:
                drive.transmit(bus, now)

            if args.status_period and now - last_status >= args.status_period:
                last_status = now
                print(
                    f"[{now - start:6.1f}s] {board.status()}"
                    + (f" | {drive.status()}" if drive else ""),
                    flush=True,
                )
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
