# Modbus RTU on the encoder board — notes for future me

What this board speaks over RS485, how to talk to it, and the sharp edges
that cost real debugging time to find. Source of truth is always the code
(`drivers/modbus_rtu.hpp`/`.cpp`, `drivers/rs485_transport.hpp`/`.cpp`,
`rtos/comms_task.cpp`) — this doc explains the _why_, the code has the
exact bytes.

## The short version

- **Transport:** Modbus RTU, RS485 half-duplex, 9600 baud, 8N1, on `uart1`
  (RX, hardware) + a PIO program (TX — see "Why TX is PIO, not hardware
  UART" below).
- **Address:** DIP switch value (0-7, printed at boot) **+ 1**. So DIP
  `000` → Modbus address `1`, DIP `011` (3) → address `4`, etc. Address 0
  is never a real board — it's reserved for broadcast.
- **Registers:** angle (2, read-only), zero command (1, write-only),
  heartbeat (1, write-only), status (1, read-only), discovery (1,
  read/write). Full table below.
- **Heartbeat:** send as a **broadcast** (address 0) write, not addressed
  to any one board. Every board on the bus picks it up in a single
  transaction. **Wait ~20ms after sending it before your next request** —
  see the gotcha below, this one is not optional. Only for debugging keep in mind, pyserial stuff, proper modbus doesn't have this issue.
- **Status LED:** full color/priority reference below — worth reading
  before assuming a given color means what you'd guess.

## Register map

All registers are 16-bit holding registers, function codes 0x03 (read),
0x06 (write single), 0x10 (write multiple — implemented, not currently
used by anything in this repo).

| Addr | Name                  | R/W | Meaning                                                                                                                                                                                                                                   |
| ---- | --------------------- | --- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 0    | `kRegAngleRaw`        | R   | AS5600 raw counts, 0-4095, after the board's zero offset is applied                                                                                                                                                                       |
| 1    | `kRegAngleDegreesX10` | R   | Same angle in degrees × 10 (e.g. `1805` = 180.5°), for convenience                                                                                                                                                                        |
| 2    | `kRegZeroCommand`     | W   | Write any nonzero value to make the _current_ position the new zero. Writing 0 is a deliberate no-op, not an error.                                                                                                                       |
| 3    | `kRegHeartbeat`       | W   | Write any value to refresh this board's "master is alive" timer. Normally sent as a broadcast — see below.                                                                                                                                |
| 4    | `kRegStatus`          | R   | Bitfield: bit0 = magnet detected, bit1 = master_alive (this board has seen a heartbeat within the last 3s)                                                                                                                                |
| 5    | `kRegDiscovery`       | R/W | Write nonzero → this board's LED goes solid bright white (identify beacon), overriding every other LED state. Write 0 → back to normal. Readable too, unlike zero/heartbeat, since it's a persistent mode rather than a one-shot trigger. |

Reading a write-only register (2 or 3) or writing a read-only one
correctly returns a Modbus exception (illegal data address), not silence
or garbage — if you get that, you asked for the wrong thing, the link
isn't broken.

## Reading the angle

Function 0x03, read 2 registers starting at address 0:

```python
rr = client.read_holding_registers(address=0, count=2, device_id=slave_addr)
raw_counts, degrees_x10 = rr.registers
degrees = degrees_x10 / 10.0
```

If no magnet is present, this comes back as an exception (illegal data
address) rather than a value — that's correct behavior (`As5600::read()`
failed), not a bus problem. Check `kRegStatus` bit0 first if you want to
distinguish "no magnet" from "board unreachable."

Note the angle you get back is **not** the sensor's live value at request
time — it's whatever `encoder_task` last published, which is refreshed at
1kHz internally on the board regardless of how often you poll it over
Modbus (see "1kHz sampling vs. Modbus polling rate" below).

## Resetting the angle to zero

Function 0x06, write any nonzero value to register 2:

```python
client.write_register(address=2, value=1, device_id=slave_addr)
```

This is a **software offset**, not a burn to the AS5600's OTP. It's undone
on power cycle, and it only affects this one board — there's no broadcast
zero (wouldn't make sense; boards aren't at the same angle).

## Heartbeat

Two independent liveness checks exist, and they're not the same
mechanism — don't conflate them:

1. **Board → detects lost master.** Each board tracks `heartbeat_seen_` /
   `last_heartbeat_ms_` internally (`ModbusRtu::master_alive()`). If it
   hasn't seen a heartbeat write in the last `kHeartbeatTimeoutMs` (3s,
   `rtos/shared_state.hpp`), `kRegStatus` bit1 goes to 0 and the status
   LED switches to its red "lost" blink. This is entirely board-local —
   it doesn't require the master to do anything except keep sending
   heartbeats.

2. **Master → detects a dead board.** Not provided by the protocol at
   all. Modbus gives you nothing for this beyond "my request timed out."
   The recommended approach (not implemented anywhere in this repo — this
   is a note for whoever writes the master side): track consecutive
   failed/timed-out polls per board, flag "stale" after ~3, require ~2
   consecutive good responses to un-flag. See the chat history around
   2026-09-19 for the full reasoning; short version is that the broadcast
   heartbeat _cannot_ be used for this, because broadcast frames never get
   a reply from anyone (see below) — per-board liveness has to come from
   the addressed angle/status polls you're already doing.

### Why heartbeat is broadcast, and why that's safe

Sent as function 0x06 to **address 0**, not to any individual board's
address. Every board on the bus executes it in one transaction instead of
needing N separate writes. Modbus RTU has **no electrical bus
arbitration** (unlike CAN) — RS485 drivers are push-pull, so if two
devices drove the bus at once you'd get real contention, not a clean
"loser backs off." The protocol avoids this entirely by convention:
**broadcast frames never get a reply, from anyone.** `ModbusRtu` (both
`handle_write_single_register` and `handle_write_multiple_registers`)
executes the write but returns before ever calling
`Rs485Transport::send()` when the frame was a broadcast — no board even
asserts its RS485 driver in response, so collision is structurally
impossible, not just unlikely.

### The 20ms gotcha (read this before you re-debug it)

**Wait at least ~20ms after sending the broadcast heartbeat before your
next request.** This bit us for real — the LED appeared stuck on "no
heartbeat" blue even though the heartbeat write reported success every
time, and it took a few hours of hardware-level debugging (a whole
sub-thread of raw-socket RS485 tests, timing pymodbus internals, checking
`no_response_expected`) to pin down.

The actual cause has nothing to do with the firmware: `pyserial`'s
`write()` (which pymodbus calls internally) returns as soon as bytes are
handed to the OS's serial buffer, **not** once they've actually finished
transmitting on the wire. An 8-byte broadcast frame takes ~8.3ms to
physically clear at 9600 baud. Because broadcast frames get no reply,
there's nothing else naturally pacing the client the way waiting for a
normal response would — so firing the next request immediately overlaps
it with the tail of the still-transmitting broadcast frame, corrupting
both at the wire level. The firmware never sees a clean frame, never
replies, and the client burns a full ~1s timeout (plus a retry) doing
nothing useful. Confirmed by direct measurement: without the delay, the
very next read after a broadcast consistently took ~1040ms and returned
stale data; with a 20ms delay inserted, the same read took ~30ms and
returned correct, current data every time.

`tools/test_encoder_modbus.py` already does this (`time.sleep(0.02)`
right after the broadcast write) — if you write a different master and
skip it, you will rediscover this exact bug.

## Status LED reference

`drivers/status_led.hpp` computes the color, `rtos/led_task.cpp` drives it.
Priority order below is highest first — only one state shows at a time,
and higher entries win when more than one would technically apply.

| Priority | State                                               | Color                 | Pattern                                                                                                                                                                                             |
| -------- | --------------------------------------------------- | --------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1        | Discovery active (`kRegDiscovery` written nonzero)  | White `(255,255,255)` | Solid on. The one color that isn't dimmed — deliberately unmissable, it's a manual "which physical board is this" beacon.                                                                           |
| 2        | Every 10th heartbeat to arrive (within 100ms of it) | Cyan                  | Brief blip, interrupts whatever else was showing, then reverts. Every single heartbeat was too frequent in practice; every 10th is a periodic "still alive" confirmation without being distracting. |
| 3        | Never seen a heartbeat yet                          | Blue                  | Blink at the heartbeat rate, **plus** a quick triple-flash every 10 seconds — a board sitting untouched for a while still visibly confirms it's running, not just slow-blinking forever.            |
| 4        | Heartbeat previously seen, now lost (timed out)     | Red                   | Blink at the heartbeat rate.                                                                                                                                                                        |
| 5        | Magnet not detected                                 | Amber                 | Blink at the heartbeat rate.                                                                                                                                                                        |
| 6        | Magnet present, moving (velocity > 0)               | Green                 | Rapid flash (4x heartbeat rate).                                                                                                                                                                    |
| 6        | Magnet present, moving (velocity < 0)               | Green                 | Flash at 2x heartbeat rate.                                                                                                                                                                         |
| 7        | Magnet present, not moving                          | —                     | **Off.** Green is motion-only now; there's no separate "alive and idle" color — the cyan heartbeat blip already covers "still connected."                                                           |

A couple of things follow from `encoder_task`'s own logic, not just LED
priority: "magnet missing" and "moving" can never be true at the same
time, because `encoder_task` only computes a nonzero velocity when the
magnet is actually present (see `encoder_task.cpp`) — so amber and green
are mutually exclusive by construction, not just by the priority table.
Likewise "just got a heartbeat" and "waiting for first heartbeat" can't
coexist: the first heartbeat ever received permanently ends the "waiting"
state, so the cyan blip and the blue pattern never fight over the same
instant.

## Discovering multiple boards on the bus

`tools/discover_encoders.py` scans Modbus addresses 1-8, and for each one
that responds, briefly switches its discovery LED to white so you can
match "address N" to a physical board, then polls all found boards'
angles round-robin (with the broadcast heartbeat kept alive throughout).
Ctrl+C to stop; it cleans up any discovery LED it left on along the way,
including if it's interrupted mid-scan.

```
python3 discover_encoders.py /dev/ttyUSB0
```

Same 20ms-after-broadcast rule applies here as everywhere else (see
above) — already handled in the script, mentioned again because it's the
easiest thing to accidentally drop if this gets copied into a new master
implementation.

## Addressing details

- DIP switch → `read_board_id()` (`drivers/board_id.hpp`) → raw 0-7,
  active-low (switch **on** = pin driven low = bit set). Printed at boot
  as `device id: N, direction: CW/CCW`.
- Modbus slave address = that value **+ 1**. Applied where `ModbusRtu` is
  constructed (`rtos/comms_task.cpp`), not inside `board_id.cpp` itself,
  since the raw 0-7 value used to also drive the status LED's now-removed
  "ack chase" pattern — kept the offset out of `board_id.cpp` on the
  principle that a GPIO-read helper shouldn't bake in a Modbus-specific
  transform, even though nothing else needs the raw value anymore.
- This means DIP `000` is a real, addressable board (address 1) — it does
  **not** collide with the broadcast address, which was the whole point
  of the +1 offset.

## 1kHz sampling vs. Modbus polling rate — these are different numbers

`encoder_task` (core 1) samples the AS5600 over I2C and publishes a fresh
reading at a true, steady 1kHz, entirely independent of the RS485 bus —
see `rtos/encoder_task.cpp`, `vTaskDelayUntil` locks this to exactly 1ms
regardless of Modbus traffic. Nothing you do on the Modbus side changes
this rate.

What Modbus _can_ deliver to a master polling over this link is a much
lower, protocol-limited number:

- At 9600 baud: ~26-28ms per transaction (8-byte request + mandatory 3.5
  inter-frame silence gaps on both sides + response), i.e. a ceiling
  around **35-40Hz** for a single register-read transaction.
- The Modbus spec fixes that inter-frame gap at a flat **1.75ms** for any
  baud above 19200 — it does not keep shrinking as baud increases, because
  typical UART/OS timing can't reliably enforce anything finer. That puts
  a **structural ceiling around ~285 transactions/sec, at any baud**,
  if you stay spec-compliant.
- This firmware's own gap timing (`ModbusRtu::poll()`'s
  `ceil(38500/baud)`) doesn't follow that fixed-1.75ms convention — it
  keeps shrinking with baud, since this is a closed system (our own
  master + our own slaves, no need to interoperate with third-party
  Modbus masters). But `Rs485Transport`/`ModbusRtu` currently work in
  **millisecond** granularity, which creates its own ~1-2ms/transaction
  floor — so a higher baud alone won't get past the _spec's_ ~285Hz
  ceiling without also tightening that timing to microsecond resolution.
- Practical target actually in use: **50Hz per board** (150Hz aggregate
  across 3 boards) — comfortably achievable with a baud bump (e.g. to
  115200-230400) alone, no timing-granularity work needed. The literal
  "100Hz × 3 boards" figure floated earlier sits right at the spec
  ceiling and isn't worth chasing unless it's actually needed.

If polling rate ever needs to go up again: bump `MODBUS_BAUD_HZ` (both
`comms_task.cpp`'s default and whatever the master uses) before touching
anything else, and validate reliability in the actual deployment
environment — this board's target application (excavation bucket joints)
sits close to hydraulics and motor drives, and higher baud trades off
noise immunity on RS485.

## Why TX is PIO, not the hardware UART

Worth knowing if you're ever staring at `rs485_transport.hpp` wondering
why RX uses `hardware_uart` but TX doesn't. GPIO7 (this board's RS485_TX
net) has no `uart1_tx` option in the RP2350's pin mux — only
`uart1_rts`/`uart1_rx` are available there (checked directly against the
SDK's register definitions, not a guess). GPIO4, the "normal" UART1 TX
pin, is already claimed by the WS2812 status LED. So TX is bit-banged via
a PIO program (`drivers/uart_tx.pio`) instead. This cost a real bug once
already — the first version had an extra untimed instruction between the
start bit and the data-bit loop that stretched the start bit by 1/8 of a
bit period, skewing every subsequent bit just enough to break framing.
Fixed now, but if RS485 output ever looks subtly wrong again (garbled
bytes, not total silence), the PIO timing is the first place to
re-derive, cycle by cycle, rather than assume it's still correct.

## Exceptions you'll actually see

Standard Modbus exception response: function code with the high bit set
(e.g. `0x03` read request → `0x83` exception reply), followed by one
reason byte.

| Code | Name                 | When                                                                                                                          |
| ---- | -------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| 0x01 | Illegal function     | Function code isn't 0x03/0x06/0x10                                                                                            |
| 0x02 | Illegal data address | Register is write-only (reading 2 or 3), read-only (writing 4), out of range, or (for angle) the AS5600 read genuinely failed |
| 0x04 | Slave device failure | Defined in the register map but not currently returned by anything — reserved, not wired to a real failure path yet           |

CRC failures and address mismatches are **silent**, per spec — no
exception, no reply at all. If a request just vanishes with zero
response, check CRC/addressing before assuming something's broken.
