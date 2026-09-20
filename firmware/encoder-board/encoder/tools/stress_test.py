#!/usr/bin/env python3
"""Long-duration stress test for one or more encoder boards' Modbus RTU +
FreeRTOS stack, sharing a single RS485 bus.

Usage:
    python3 stress_test.py /dev/ttyUSB0 \\
        [--device-ids 0,1,2] [--duration SECONDS] \\
        [--debug-port /dev/ttyACM0 [--debug-port /dev/ttyACM1 ...]]

With no --device-ids, scans addresses 1-8 first (same approach as
discover_encoders.py) and stress-tests whatever responds. Each iteration
of the main loop picks a random board *and* a random action, so this
exercises real multi-drop bus behavior -- addressing correctness, one
board's requests not disturbing another's, one board's crash not being
masked by others still answering -- not just one board in isolation.

Crash detection is tracked **per board**, not globally: if board A stops
responding while B and C keep answering fine, B/C's successes must not
reset A's failure streak, or a dead board could hide behind healthy ones
indefinitely. See Stats below.

If one or more --debug-port is given (each board's own USB serial
console, separate from the RS485 adapter -- typically /dev/ttyACM0,
/dev/ttyACM1, ...), a background thread per port tails it for firmware
fault messages (e.g. vApplicationStackOverflowHook's "FATAL: stack
overflow in task ..." from encoder.cpp) and tags any hit with which port
it came from. This is the strongest direct crash signal available, since
a hung comms_task leaves Modbus silent without saying why.

Exit code is 0 if nothing unexpected happened, 1 otherwise -- suitable
for unattended runs (cron, CI, an overnight soak), checked by exit code
alone.
"""

import argparse
import random
import sys
import threading
import time

from pymodbus.client import ModbusSerialClient

# Register map -- must match drivers/modbus_rtu.hpp's ModbusRegister enum.
REG_ANGLE_RAW = 0
REG_ANGLE_DEGREES_X10 = 1
REG_ZERO_COMMAND = 2
REG_HEARTBEAT = 3
REG_STATUS = 4
REG_DISCOVERY = 5
REG_OUT_OF_RANGE = 99

BAUD = 9600  # must match MODBUS_BAUD_HZ in CMakeLists.txt
MIN_ADDR = 1
MAX_ADDR = 8  # DIP switch 0-7 + 1; address 0 is reserved for broadcast

HEARTBEAT_INTERVAL_S = 0.5  # well under kHeartbeatTimeoutMs (3s) in shared_state.hpp
CRASH_CONSECUTIVE_FAILURES = (
    10  # genuine no-response timeouts in a row -> "probably crashed"
)
FAULT_KEYWORDS = ("fatal", "assert", "hard fault", "panic")


class Stats:
    """Thread-safe: written from the main loop and every debug-UART thread."""

    def __init__(self):
        self._lock = threading.Lock()
        self._counts = {}  # (addr, action, outcome) -> count
        self._consecutive_failures = {}  # addr -> count, tracked separately per board --
        self._max_consecutive_failures = {}  # see the module docstring for why
        self._crash_events = []  # list of (monotonic_time, description)

    def record(self, addr, action, outcome):
        with self._lock:
            key = (addr, action, outcome)
            self._counts[key] = self._counts.get(key, 0) + 1
            if outcome == "unexpected_failure":
                cf = self._consecutive_failures.get(addr, 0) + 1
                self._consecutive_failures[addr] = cf
                self._max_consecutive_failures[addr] = max(
                    self._max_consecutive_failures.get(addr, 0), cf
                )
                if cf == CRASH_CONSECUTIVE_FAILURES:
                    self._crash_events.append(
                        (
                            time.monotonic(),
                            f"addr {addr}: {CRASH_CONSECUTIVE_FAILURES} consecutive "
                            f"unanswered requests",
                        )
                    )
            else:
                self._consecutive_failures[addr] = 0

    def note_fault_line(self, source, line):
        with self._lock:
            self._crash_events.append((time.monotonic(), f"{source}: {line.strip()}"))

    def snapshot(self):
        with self._lock:
            return (
                dict(self._counts),
                list(self._crash_events),
                dict(self._max_consecutive_failures),
            )


def watch_debug_uart(port, stats, stop_event):
    """Background thread: tail one board's own USB serial for fault
    keywords. One of these runs per --debug-port given."""
    import serial

    try:
        set = serial.Serial(port, baudrate=115200, timeout=0.5)
    except Exception as exc:  # noqa: BLE001
        print(f"couldn't open debug port {port}: {exc}")
        return

    while not stop_event.is_set():
        try:
            raw = set.readline()
        except Exception:  # noqa: BLE001 -- transient USB hiccups shouldn't kill this thread
            continue
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace")
        if any(kw in line.lower() for kw in FAULT_KEYWORDS):
            print(f"!! FAULT on {port}: {line.strip()}")
            stats.note_fault_line(port, line)

    set.close()


def discover(client):
    """Scan MIN_ADDR..MAX_ADDR for responsive boards. Short timeout/retries
    on the connection (set by the caller) keeps this fast -- pymodbus's
    defaults would spend up to ~12s per unpopulated address."""
    found = []
    print(f"-- scanning addresses {MIN_ADDR}-{MAX_ADDR} --")
    for addr in range(MIN_ADDR, MAX_ADDR + 1):
        try:
            rr = client.read_holding_registers(
                address=REG_STATUS, count=1, device_id=addr
            )
        except Exception:  # noqa: BLE001 -- expected for every unpopulated address
            continue
        if rr.isError():
            continue
        found.append(addr)
        print(f"  address {addr}: found (status={rr.registers[0]})")
    if not found:
        print("  no boards found")
    else:
        print(f"-- found {len(found)} board(s): {found} --")
    return found


def verify_addrs(client, addrs):
    """Confirm a caller-supplied --device-ids list actually responds,
    rather than including silent addresses that would immediately trip
    crash detection for a board that was never plugged in to begin with."""
    confirmed = []
    for addr in addrs:
        try:
            rr = client.read_holding_registers(
                address=REG_STATUS, count=1, device_id=addr
            )
        except Exception:  # noqa: BLE001
            print(f"  address {addr}: no response, excluding from the test")
            continue
        if rr.isError():
            print(f"  address {addr}: no response, excluding from the test")
            continue
        confirmed.append(addr)
        print(f"  address {addr}: confirmed (status={rr.registers[0]})")
    return confirmed


def classify(call):
    """Run `call` (a zero-arg lambda wrapping a pymodbus request), returning
    ("ok", result), ("expected_exception", result), or
    ("unexpected_failure", exc). A Modbus exception response (e.g. illegal
    data address) is a normal, well-formed reply -- not a failure -- so
    it's distinguished from a genuine no-response timeout, which pymodbus
    surfaces as a raised exception rather than an error response."""
    try:
        result = call()
    except Exception as exc:  # noqa: BLE001
        return "unexpected_failure", exc
    if result.isError():
        return "expected_exception", result
    return "ok", result


def do_heartbeat(client, stats):
    # Broadcast reaches every board on the bus in one transaction -- no
    # per-board loop needed here, unlike every other action. The mandatory
    # post-write settle delay applies regardless of how many boards are
    # listening; see docs/modbus.md for why it's not optional.
    try:
        client.write_register(
            address=REG_HEARTBEAT, value=1, device_id=0, no_response_expected=True
        )
        time.sleep(0.02)
        stats.record("broadcast", "heartbeat", "ok")
    except Exception:  # noqa: BLE001
        stats.record("broadcast", "heartbeat", "unexpected_failure")


def action_read_angle(client, addr):
    outcome, _ = classify(
        lambda: client.read_holding_registers(
            address=REG_ANGLE_RAW, count=2, device_id=addr
        )
    )
    return outcome  # "expected_exception" correctly covers "no magnet on the bench"


def action_read_status(client, addr):
    outcome, _ = classify(
        lambda: client.read_holding_registers(
            address=REG_STATUS, count=1, device_id=addr
        )
    )
    return outcome


def action_read_discovery(client, addr):
    outcome, _ = classify(
        lambda: client.read_holding_registers(
            address=REG_DISCOVERY, count=1, device_id=addr
        )
    )
    return outcome


def action_toggle_discovery(client, addr):
    value = random.choice([0, 1])
    outcome, _ = classify(
        lambda: client.write_register(
            address=REG_DISCOVERY, value=value, device_id=addr
        )
    )
    if outcome != "ok":
        return outcome

    # Verify it actually stuck -- kRegDiscovery is readable specifically
    # so this kind of check is possible.
    check_outcome, result = classify(
        lambda: client.read_holding_registers(
            address=REG_DISCOVERY, count=1, device_id=addr
        )
    )
    if check_outcome != "ok" or result.registers[0] != (1 if value else 0):
        return "unexpected_failure"  # wrote one value, read back another (or couldn't read at all)
    return "ok"


def action_zero_command(client, addr):
    value = random.choice(
        [0, 1]
    )  # exercises both "real zero" and the "0 = no-op" contract
    outcome, _ = classify(
        lambda: client.write_register(
            address=REG_ZERO_COMMAND, value=value, device_id=addr
        )
    )
    return outcome


def action_write_multiple(client, addr):
    # Function code 0x10 -- implemented but not exercised by anything else
    # in this repo. Writes a no-op zero command through that path.
    outcome, _ = classify(
        lambda: client.write_registers(
            address=REG_ZERO_COMMAND, values=[0], device_id=addr
        )
    )
    return outcome


def action_read_out_of_range(client, addr):
    outcome, _ = classify(
        lambda: client.read_holding_registers(
            address=REG_OUT_OF_RANGE, count=1, device_id=addr
        )
    )
    if outcome == "ok":
        return "unexpected_failure"  # a register that doesn't exist shouldn't ever "succeed"
    return outcome  # expected_exception (normal) or unexpected_failure (timeout)


def action_read_write_only(client, addr):
    reg = random.choice([REG_ZERO_COMMAND, REG_HEARTBEAT])
    outcome, _ = classify(
        lambda: client.read_holding_registers(address=reg, count=1, device_id=addr)
    )
    if outcome == "ok":
        return "unexpected_failure"  # a write-only register that let itself be read is a real bug
    return outcome


def action_write_read_only(client, addr):
    reg = random.choice([REG_ANGLE_RAW, REG_ANGLE_DEGREES_X10, REG_STATUS])
    outcome, _ = classify(
        lambda: client.write_register(address=reg, value=1, device_id=addr)
    )
    if outcome == "ok":
        return "unexpected_failure"  # a read-only register that accepted a write is a real bug
    return outcome


# (name, relative weight, fn) -- weight is arbitrary units, not a percentage.
ACTIONS = [
    ("read_angle", 30, action_read_angle),
    ("read_status", 20, action_read_status),
    ("read_discovery", 5, action_read_discovery),
    ("toggle_discovery", 5, action_toggle_discovery),
    ("zero_command", 5, action_zero_command),
    ("write_multiple", 3, action_write_multiple),
    ("read_out_of_range", 5, action_read_out_of_range),
    ("read_write_only", 5, action_read_write_only),
    ("write_read_only", 5, action_write_read_only),
]
_ACTION_CHOICES = [(name, fn) for name, _weight, fn in ACTIONS]
_ACTION_WEIGHTS = [weight for _name, weight, _fn in ACTIONS]


def run(client, addrs, duration_s, stats, start):
    next_heartbeat = start
    next_report = (
        start + 60
    )  # a 30-minute silent script is indistinguishable from a hung one

    while time.monotonic() - start < duration_s:
        now = time.monotonic()

        if now >= next_heartbeat:
            do_heartbeat(client, stats)
            next_heartbeat += HEARTBEAT_INTERVAL_S

        addr = random.choice(addrs)
        name, fn = random.choices(_ACTION_CHOICES, weights=_ACTION_WEIGHTS, k=1)[0]
        stats.record(addr, name, fn(client, addr))

        if now >= next_report:
            counts, crash_events, _max_consec = stats.snapshot()
            total = sum(counts.values())
            print(
                f"[{(now - start) / 60:5.1f} min] {total} requests sent, "
                f"{len(crash_events)} crash event(s) so far"
            )
            next_report += 60


def print_summary(stats, start, duration_s, addrs):
    counts, crash_events, max_consec = stats.snapshot()
    total = sum(counts.values())

    print()
    print("=" * 60)
    print("STRESS TEST SUMMARY")
    print("=" * 60)
    print(
        f"duration: {duration_s / 60:.1f} min, boards tested: {addrs}, total requests: {total}"
    )
    # key=... : addr is an int for a per-board action but the string
    # "broadcast" for heartbeats -- sort by string form so the two types
    # never get compared directly against each other.
    for (addr, action, outcome), count in sorted(
        counts.items(), key=lambda kv: (str(kv[0][0]), kv[0][1], kv[0][2])
    ):
        print(f"  addr {addr!s:>8s}  {action:20s} {outcome:20s} {count}")
    print("max consecutive unanswered requests per board:")
    for addr in addrs:
        print(f"  addr {addr}: {max_consec.get(addr, 0)}")
    print(f"crash events: {len(crash_events)}")
    for event_time, desc in crash_events:
        print(f"  [{(event_time - start) / 60:6.1f} min] {desc}")

    unexpected = sum(
        c for (_a, _act, o), c in counts.items() if o == "unexpected_failure"
    )
    if crash_events:
        print("\nRESULT: FAIL -- possible crash/hang detected, see crash events above")
        return 1
    if unexpected:
        print(
            f"\nRESULT: FAIL -- {unexpected} unexpected failure(s), see breakdown above"
        )
        return 1
    print("\nRESULT: PASS")
    return 0


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("port", help="RS485 adapter serial port, e.g. /dev/ttyUSB0")
    parser.add_argument(
        "--device-ids",
        default=None,
        help="comma-separated DIP-switch device ids to test, e.g. 0,1,2. "
        "Default: scan addresses 1-8 and test whatever responds.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=1800,
        help="seconds to run (default 1800 = 30 min)",
    )
    parser.add_argument(
        "--debug-port",
        action="append",
        default=[],
        help="a board's own USB serial, e.g. /dev/ttyACM0 -- repeatable, one per board. "
        "Optional but recommended.",
    )
    args = parser.parse_args()

    # timeout=0.3, retries=1: fast enough to make scanning/verifying
    # practical -- pymodbus's defaults (timeout=3, retries=3) would burn
    # up to ~12s per address with nothing attached. Still generous for a
    # real ~25-30ms round trip at 9600 baud.
    client = ModbusSerialClient(
        port=args.port,
        baudrate=BAUD,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=0.3,
        retries=1,
    )
    if not client.connect():
        print(f"couldn't open {args.port}")
        sys.exit(1)

    if args.device_ids:
        requested = [int(d) + 1 for d in args.device_ids.split(",")]
        print(f"-- verifying requested device ids {args.device_ids} --")
        addrs = verify_addrs(client, requested)
    else:
        addrs = discover(client)

    if not addrs:
        print("no responsive boards -- nothing to test")
        client.close()
        sys.exit(1)

    stats = Stats()
    stop_event = threading.Event()
    debug_threads = []
    for debug_port in args.debug_port:
        t = threading.Thread(
            target=watch_debug_uart, args=(debug_port, stats, stop_event), daemon=True
        )
        t.start()
        debug_threads.append(t)
        print(f"-- watching {debug_port} for firmware fault messages --")
    if not args.debug_port:
        print(
            "-- no --debug-port given: crash detection relies on Modbus responsiveness alone --"
        )

    print(
        f"-- stress testing {len(addrs)} board(s) {addrs} for {args.duration / 60:.1f} min --"
    )

    start = time.monotonic()
    try:
        run(client, addrs, args.duration, stats, start)
    except KeyboardInterrupt:
        print("\n-- stopped early --")
    finally:
        stop_event.set()
        client.close()
        for t in debug_threads:
            t.join(timeout=1)

    sys.exit(print_summary(stats, start, time.monotonic() - start, addrs))


if __name__ == "__main__":
    main()
