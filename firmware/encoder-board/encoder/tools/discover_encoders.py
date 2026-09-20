#!/usr/bin/env python3
"""Discover all encoder boards on the RS485 bus and monitor their angles.

Usage:
    python3 discover_encoders.py /dev/ttyUSB0

Scans Modbus addresses 1-8 (the DIP switch's device-id 0-7 range, +1 -- see
docs/modbus.md) for boards that respond. Each board found gets its
discovery LED (register 5, kRegDiscovery) briefly switched to solid white
so you can visually match "address N" to a physical board, then it's
switched back off. Once the scan is done, all found boards are polled for
angle round-robin while the periodic broadcast heartbeat keeps their
status LEDs (and master_alive) live. Runs until Ctrl+C.
"""

import sys
import time

from pymodbus.client import ModbusSerialClient

# Register map -- must match drivers/modbus_rtu.hpp's ModbusRegister enum.
REG_ANGLE_RAW = 0
REG_ANGLE_DEGREES_X10 = 1
REG_ZERO_COMMAND = 2
REG_HEARTBEAT = 3
REG_STATUS = 4
REG_DISCOVERY = 5

BAUD = 9600  # must match MODBUS_BAUD_HZ in CMakeLists.txt
MIN_ADDR = 1
MAX_ADDR = 8  # DIP switch 0-7 + 1; address 0 is reserved for broadcast

IDENTIFY_FLASH_S = 0.6  # how long each found board's white LED stays on during the scan
HEARTBEAT_INTERVAL_S = 0.5  # well under kHeartbeatTimeoutMs (3s) in shared_state.hpp
POLL_INTERVAL_S = (
    0.25  # ~4Hz aggregate is safely sustainable at 9600 baud; see docs/modbus.md
)


def discover(client, identified):
    """Scan MIN_ADDR..MAX_ADDR for responsive boards, briefly lighting each
    one's discovery LED as it's found. `identified` is appended to as each
    board is found, so a caller-level try/finally can always clean up
    whatever's been touched so far, even if this is interrupted partway."""
    found = []
    print(f"-- scanning addresses {MIN_ADDR}-{MAX_ADDR} --")
    for addr in range(MIN_ADDR, MAX_ADDR + 1):
        # An address with no board attached doesn't return an error
        # response -- pymodbus raises after it exhausts its retries
        # waiting for a reply that's never coming. Both cases mean
        # "nothing here, move on."
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

        client.write_register(address=REG_DISCOVERY, value=1, device_id=addr)
        identified.append(addr)
        time.sleep(IDENTIFY_FLASH_S)
        client.write_register(address=REG_DISCOVERY, value=0, device_id=addr)
        identified.remove(addr)

    if not found:
        print("  no boards found")
    else:
        print(f"-- found {len(found)} board(s): {found} --")
    return found


def send_heartbeat(client):
    # Broadcast (device_id=0): every board picks this up in one
    # transaction, and broadcast frames never get a reply (see
    # docs/modbus.md), so no_response_expected=True is required -- without
    # it pymodbus sits out a full timeout waiting for a reply that never
    # comes. The 20ms sleep afterward is not optional either: it's the
    # time the 8-byte broadcast frame needs to actually finish
    # transmitting at 9600 baud. Skip it and the *next* request gets
    # corrupted at the wire and silently fails -- this cost real debugging
    # time to track down once already, see docs/modbus.md for the full story.
    try:
        client.write_register(
            address=REG_HEARTBEAT, value=1, device_id=0, no_response_expected=True
        )
        time.sleep(0.02)
    except Exception as exc:  # noqa: BLE001 -- just reporting, not handling
        print(f"heartbeat broadcast failed: {exc}")


def monitor(client, addrs):
    print(
        f"-- monitoring {len(addrs)} board(s), heartbeat every {HEARTBEAT_INTERVAL_S}s "
        f"(Ctrl+C to stop) --"
    )
    start = time.monotonic()
    next_heartbeat = start
    next_poll = start

    while True:
        now = time.monotonic()
        if now >= next_heartbeat:
            send_heartbeat(client)
            next_heartbeat += HEARTBEAT_INTERVAL_S

        if now >= next_poll:
            elapsed = now - start
            readings = []
            for addr in addrs:
                # A board that's genuinely gone (unplugged, wire fault)
                # raises here rather than returning an error response --
                # same reasoning as discover(). Don't let one dropped
                # board crash monitoring of the rest.
                try:
                    rr = client.read_holding_registers(
                        address=REG_ANGLE_RAW, count=2, device_id=addr
                    )
                except Exception:  # noqa: BLE001
                    readings.append(f"#{addr}: no response")
                    continue
                if rr.isError():
                    readings.append(f"#{addr}: err")
                else:
                    _raw, deg_x10 = rr.registers
                    readings.append(f"#{addr}: {deg_x10 / 10.0:6.1f} deg")
            print(f"[{elapsed:6.1f}s] " + "  ".join(readings))
            next_poll += POLL_INTERVAL_S

        time.sleep(0.001)


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(1)

    port = sys.argv[1]
    # timeout=0.3, retries=1: a real 9600-baud round trip is ~25-30ms, so
    # this is still generous for boards that are actually there, but keeps
    # the scan fast -- pymodbus's default (timeout=3, retries=3) would
    # burn up to ~12s per unpopulated address, and most of the 1-8 range
    # is unpopulated on any bus with fewer than 8 boards.
    client = ModbusSerialClient(
        port=port,
        baudrate=BAUD,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=0.3,
        retries=1,
    )
    if not client.connect():
        print(f"couldn't open {port}")
        sys.exit(1)

    # Addresses whose discovery LED might currently be on -- discover()
    # keeps this current as it goes, so if we're interrupted mid-scan we
    # still know what to turn back off.
    identified = []

    try:
        addrs = discover(client, identified)
        if addrs:
            monitor(client, addrs)
    except KeyboardInterrupt:
        print("\n-- stopped --")
    finally:
        for addr in identified:
            try:
                client.write_register(address=REG_DISCOVERY, value=0, device_id=addr)
            except Exception:  # noqa: BLE001 -- best-effort cleanup on the way out
                pass
        client.close()


if __name__ == "__main__":
    main()
