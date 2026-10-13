#!/usr/bin/env python3
"""Bring-up test for the encoder board's Modbus RTU slave.

Usage:
    python3 test_encoder_modbus.py /dev/ttyUSB0 <device_id 0-7>

device_id is the DIP-switch value printed by the board at boot
("device id: N, direction: ..."); the Modbus slave address is device_id + 1
(see the comment where the slave is constructed in rtos/comms_task.cpp for why).

Note on sample rate: the firmware's encoder_task samples the AS5600 over
I2C at 1kHz internally (see encoder_task.cpp), but that's not the same as
how fast *this script* can poll it over Modbus RTU -- a single register
read round-trip at 115200 baud (request + response + RS485 turnaround) takes
a few milliseconds, plus USB-serial adapter and pymodbus overhead, so the
rate on this side of the link is well below the firmware's internal rate. This script polls at
a safely sustainable rate and reports what it actually achieved rather
than claiming a number the bus can't deliver.

Note on the heartbeat: it's sent as a Modbus broadcast (device_id=0), not
addressed to this one board -- every device on the bus picks it up in a
single transaction, and per the Modbus spec broadcast frames never get a
reply, so there's no risk of multiple boards trying to drive the RS485
bus at once (Modbus has no electrical arbitration; "broadcast gets no
reply" is the entire mechanism that prevents that collision).
"""

import sys
import time

from pymodbus.client import ModbusSerialClient

# Register map -- must match modbus::profiles::encoder::Register in firmware/shared/modbus-core/include/modbus/profiles/encoder.hpp.
REG_ANGLE_RAW = 0
REG_ANGLE_DEGREES_X10 = 1
REG_ZERO_COMMAND = 2
REG_HEARTBEAT = 3
REG_STATUS = 4

BAUD = 115200  # must match MODBUS_BAUD_HZ in rtos/comms_task.cpp

RUN_DURATION_S = 20
HEARTBEAT_INTERVAL_S = 0.5  # well under kHeartbeatTimeoutMs (3s) in shared_state.hpp
POLL_INTERVAL_S = (
    0.25  # ~4Hz -- comfortably sustainable at 115200 baud; see module docstring
)


def sanity_checks(client, modbus_addr):
    """Quick one-shot checks: zero command and an out-of-range register."""
    print("-- sending zero command (register 2) --")
    wr = client.write_register(address=REG_ZERO_COMMAND, value=1, device_id=modbus_addr)
    print("  ok" if not wr.isError() else f"  failed: {wr}")

    time.sleep(0.2)
    rr = client.read_holding_registers(
        address=REG_ANGLE_RAW, count=1, device_id=modbus_addr
    )
    if not rr.isError():
        print(f"  angle raw after zero: {rr.registers[0]} (should be near 0)")

    print("-- probing an out-of-range register (expect an exception, not silence) --")
    rr = client.read_holding_registers(address=99, count=1, device_id=modbus_addr)
    print(f"  {rr}")


def run_monitor(client, modbus_addr):
    """Poll the angle continuously for RUN_DURATION_S, refreshing the
    heartbeat throughout so master_alive stays live for the whole run (the
    status LED's cyan blip should be visible on each heartbeat, per
    docs/modbus.md's LED reference)."""
    print(
        f"-- monitoring for {RUN_DURATION_S}s, heartbeat every {HEARTBEAT_INTERVAL_S}s --"
    )

    start = time.monotonic()
    next_heartbeat = start
    next_poll = start
    poll_count = 0
    error_count = 0
    last_status = None

    while True:
        now = time.monotonic()
        elapsed = now - start
        if elapsed >= RUN_DURATION_S:
            break

        if now >= next_heartbeat:
            # Broadcast (device_id=0): every board on the bus picks this up
            # in one transaction instead of one write per device, and per
            # the Modbus spec broadcast frames never get a reply -- no
            # device asserts its RS485 driver in response, so there's no
            # risk of bus contention. no_response_expected=True stops
            # pymodbus from sitting out its full timeout waiting for a
            # reply that (correctly) never arrives.
            try:
                client.write_register(
                    address=REG_HEARTBEAT,
                    value=1,
                    device_id=0,
                    no_response_expected=True,
                )
                # pyserial's write() (which pymodbus calls internally) returns
                # once bytes are handed to the OS buffer, not once they've
                # actually finished transmitting on the wire (~0.8ms for this
                # 8-byte frame at 115200 baud, plus adapter latency). Firing the next request
                # immediately corrupts its timing at the wire -- confirmed by
                # direct testing: without this delay, the very next request
                # gets no response and pymodbus burns a full retry timeout
                # recovering. 20ms gives comfortable margin.
                time.sleep(0.02)
            except Exception as exc:  # noqa: BLE001 -- just reporting, not handling
                print(f"  [{elapsed:5.1f}s] heartbeat broadcast failed: {exc}")
            next_heartbeat += HEARTBEAT_INTERVAL_S

        if now >= next_poll:
            # Status is read unconditionally, not gated behind the angle
            # read succeeding -- master_alive/magnet_detected are relevant
            # even when there's no magnet on the bench and the angle read
            # itself is (correctly) returning an exception.
            sr = client.read_holding_registers(
                address=REG_STATUS, count=1, device_id=modbus_addr
            )
            status = sr.registers[0] if not sr.isError() else None
            last_status = status if status is not None else last_status
            status_str = (
                f"magnet={bool(status & 1)} alive={bool(status & 2)}"
                if status is not None
                else "status read failed"
            )

            rr = client.read_holding_registers(
                address=REG_ANGLE_RAW, count=2, device_id=modbus_addr
            )
            if rr.isError():
                error_count += 1
                print(f"  [{elapsed:5.1f}s] angle read failed: {rr}  {status_str}")
            else:
                poll_count += 1
                raw, deg_x10 = rr.registers
                print(
                    f"  [{elapsed:5.1f}s] angle raw={raw:4d} deg={deg_x10 / 10.0:6.1f}  {status_str}"
                )
            next_poll += POLL_INTERVAL_S

        time.sleep(0.001)

    achieved_hz = poll_count / RUN_DURATION_S
    print(
        f"-- done: {poll_count} successful polls, {error_count} errors, "
        f"{achieved_hz:.1f} Hz achieved poll rate over {RUN_DURATION_S}s --"
    )
    if last_status is not None:
        print(
            f"  final status: magnet_detected={bool(last_status & 1)} master_alive={bool(last_status & 2)}"
        )


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)

    port = sys.argv[1]
    device_id = int(sys.argv[2])
    modbus_addr = device_id + 1

    client = ModbusSerialClient(
        port=port, baudrate=BAUD, bytesize=8, parity="N", stopbits=1, timeout=1
    )
    if not client.connect():
        print(f"couldn't open {port}")
        sys.exit(1)

    sanity_checks(client, modbus_addr)
    run_monitor(client, modbus_addr)

    client.close()


if __name__ == "__main__":
    main()
