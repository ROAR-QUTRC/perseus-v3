# Encoder Board: Modbus Interface

Reference for the encoder board's RS485 / Modbus RTU interface, status LED and
firmware task structure. The source is authoritative; this document summarises it.

## 1. Overview

| Item           | Value                                                          |
| -------------- | -------------------------------------------------------------- |
| Protocol       | Modbus RTU slave, function codes 0x03, 0x06, 0x10              |
| Physical layer | RS485, half-duplex, 115200 baud, 8N1 (`MODBUS_BAUD_HZ`)        |
| Pins           | RX GPIO5 (UART1), DIR GPIO6 (DE/~RE), TX GPIO7 (PIO)           |
| Slave address  | DIP switch value (0-7) + 1, giving 1-8. Address 0 is broadcast |
| Broadcast      | Executed by every board; no board replies                      |

TX is driven by a PIO program (`uart_tx.pio`) because GPIO7 has no `uart1_tx`
function on the RP2350 and GPIO4 is used by the status LED.

### Shared libraries

The protocol and transport are not part of this project. They are shared with
other controllers, for example the ESP32 master on the SBB:

| Library                       | Provides                                                                                          |
| ----------------------------- | ------------------------------------------------------------------------------------------------- |
| `firmware/shared/modbus-core` | Modbus RTU framing, `modbus::Slave`, `modbus::Master`, `MBUS` scheduler, `HeartbeatTracker`       |
|                               | `modbus/profiles/encoder.hpp`: the register map below, the single source of truth                 |
| `firmware/shared/rs485`       | `rs485::Port` interface and the RP2350 backend `rs485::Rp2350Port` (UART RX, PIO TX, DIR control) |

The Python tools in `tools/` mirror the register map by hand. The API, the
health model and worked examples are in `firmware/shared/modbus-core/README.md`.

## 2. Register Map

All registers are 16-bit holding registers.

| Address | Name                  | Access | Description                                                                 |
| ------- | --------------------- | ------ | --------------------------------------------------------------------------- |
| 0       | `kRegAngleRaw`        | R      | AS5600 counts, 0-4095, after the zero offset                                |
| 1       | `kRegAngleDegreesX10` | R      | Angle in degrees x 10 (1805 = 180.5 degrees)                                |
| 2       | `kRegZeroCommand`     | W      | Nonzero: make the current position the new zero. Zero: no-op                |
| 3       | `kRegHeartbeat`       | W      | Any value refreshes the master-alive timer                                  |
| 4       | `kRegStatus`          | R      | Bit 0: magnet detected. Bit 1: master alive (heartbeat within the last 3 s) |
| 5       | `kRegDiscovery`       | R/W    | Nonzero: status LED solid white. Zero: normal operation                     |

- **Angle.** The value is the last sample published by `encoder_task` (section 4),
  not a live read. If the magnet is absent or the I2C read failed, the read
  returns exception 0x02.
- **Zero.** A software offset. It is lost on power cycle and affects only the
  addressed board. A broadcast zero is not meaningful.
- **Heartbeat.** Sent as a broadcast write to address 0. A board reports the master
  as lost after `kHeartbeatTimeoutMs` (3 s) without one. Detecting a dead board is
  the master's responsibility and relies on its own poll failures.
- **Access.** Registers 2 and 3 are write-only and register 4 is read-only, so
  registers 2 and 3 cannot be read as part of a block read.

### Exceptions

| Code | Name                 | Returned when                                                                            |
| ---- | -------------------- | ---------------------------------------------------------------------------------------- |
| 0x01 | Illegal function     | The function code is not 0x03, 0x06 or 0x10                                              |
| 0x02 | Illegal data address | Writing a read-only register, reading a write-only or unmapped one, or angle unavailable |

Frames with a bad CRC or another slave's address receive no reply. Broadcast
requests never receive a reply, including exceptions.

### Master requirements

- Wait at least the RTU frame gap (3.5 character times) between frames. The shared
  `modbus::Master` does this itself.
- A `pyserial` master must additionally wait about 20 ms after a broadcast, because
  `write()` returns before the bytes have left the wire and the next request would
  otherwise corrupt the tail of the broadcast.

## 3. Status LED

One WS2812 (GPIO4). Only the highest-priority applicable state is shown. Colours
are dim, except discovery. The blink period equals the heartbeat period (1 s).

| Priority | State                                          | Colour | Pattern                                                     |
| -------- | ---------------------------------------------- | ------ | ----------------------------------------------------------- |
| 1        | Discovery active (`kRegDiscovery` nonzero)     | White  | Solid, full brightness                                      |
| 2        | Every 10th heartbeat, for 100 ms after arrival | Cyan   | Brief blip, then returns to the underlying state            |
| 3        | No heartbeat received yet                      | Blue   | Blink at the heartbeat rate, plus a triple flash every 10 s |
| 4        | Heartbeat received previously, now timed out   | Red    | Blink at the heartbeat rate                                 |
| 5        | Magnet not detected                            | Amber  | Blink at the heartbeat rate                                 |
| 6        | Magnet present, velocity positive              | Green  | Flash at 4x the heartbeat rate                              |
| 6        | Magnet present, velocity negative              | Green  | Flash at 2x the heartbeat rate                              |
| 7        | Magnet present, stationary                     | Off    |                                                             |

States 5 and 6 are mutually exclusive: `encoder_task` reports a nonzero velocity
only while the magnet is detected.

## 4. RTOS Structure

FreeRTOS SMP on both RP2350 cores, 1 kHz tick, 64 KiB heap, stack overflow
checking enabled.

| Task           | Core | Priority | Stack (words) | Role                                                    |
| -------------- | ---- | -------- | ------------- | ------------------------------------------------------- |
| `encoder_task` | 1    | idle + 2 | 1024          | Sole owner of the AS5600. Samples at 1 kHz              |
| `comms_task`   | 0    | idle + 1 | 1024          | Runs `modbus::Slave`; answers requests from the samples |
| `led_task`     | 0    | idle + 3 | 512           | Drives the status LED every 20 ms                       |

- **Priority order.** `led_task` must outrank `comms_task`. `Rp2350Port::receive()`
  busy-waits, so an equal or lower priority `led_task` could be starved.
- **Sampling rate.** `encoder_task` samples at 1 kHz regardless of Modbus traffic
  (`vTaskDelayUntil`). Velocity direction is taken over a 50 ms window with a
  2-count deadband. The rate a master can poll is limited by the bus, not by the
  sampling rate.
- **Inter-task data.** Tasks exchange data only through the queues in `SharedState`.
  Length-1 queues use overwrite and peek, so they always hold the latest value.

| Queue              | Type             | Length | Producer       | Consumer                 |
| ------------------ | ---------------- | ------ | -------------- | ------------------------ |
| `latest_sample`    | `EncoderSample`  | 1      | `encoder_task` | `comms_task`, `led_task` |
| `encoder_commands` | `EncoderCommand` | 4      | `comms_task`   | `encoder_task`           |
| `heartbeat_state`  | `HeartbeatState` | 1      | `comms_task`   | `led_task`               |
| `discovery_active` | `bool`           | 1      | `comms_task`   | `led_task`               |

## 5. Tools

| Script                   | Purpose                                                              |
| ------------------------ | -------------------------------------------------------------------- |
| `test_encoder_modbus.py` | Bring-up test: reads angle and status from one board                 |
| `discover_encoders.py`   | Scans addresses 1-8, flashes each board's LED white, then polls them |
| `stress_test.py`         | Long-running multi-board stress test with per-board crash detection  |
| `watch_boot.py`          | Streams the USB debug console and flags faults, across power cycles  |
