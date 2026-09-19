#!/usr/bin/env python3
"""Wait for a board's USB debug console and stream everything it prints,
flagging firmware fault lines, surviving the port disappearing/reappearing
(e.g. during a power cycle).

Usage:
    python3 watch_boot.py /dev/ttyACM0 [--duration SECONDS]

Useful for catching a board's boot sequence -- including a crash, if one
happens -- when you can't precisely time a physical power cycle against a
fixed capture window: this just waits, tolerates the port vanishing and
reappearing, and keeps streaming with timestamps. Power-cycle the board
whenever you're ready after starting this; there's no window to miss.
Ctrl+C to stop early.
"""

import argparse
import time

import serial

FAULT_KEYWORDS = ("fatal", "assert", "hard fault", "panic")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("port", help="board's USB debug console, e.g. /dev/ttyACM0")
    parser.add_argument("--duration", type=float, default=60, help="seconds to run (default 60)")
    args = parser.parse_args()

    deadline = time.monotonic() + args.duration
    print(f"-- watching {args.port} for up to {args.duration:.0f}s (Ctrl+C to stop) --")
    print("-- power-cycle the board whenever you're ready; this will wait through it --")

    ser = None
    try:
        while time.monotonic() < deadline:
            if ser is None:
                try:
                    ser = serial.Serial(args.port, baudrate=115200, timeout=0.5)
                    print(f"[{time.monotonic():7.1f}s] -- port open --")
                except Exception:  # noqa: BLE001 -- keep retrying until it appears
                    time.sleep(0.2)
                    continue
            try:
                raw = ser.readline()
            except Exception as exc:  # noqa: BLE001 -- port vanished mid-read; wait and retry
                print(f"[{time.monotonic():7.1f}s] -- port lost: {exc} --")
                ser = None
                time.sleep(0.2)
                continue
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").rstrip()
            flag = "  <<< FAULT" if any(kw in line.lower() for kw in FAULT_KEYWORDS) else ""
            print(f"[{time.monotonic():7.1f}s] {line}{flag}")
    except KeyboardInterrupt:
        print("\n-- stopped --")
    finally:
        if ser:
            ser.close()


if __name__ == "__main__":
    main()
