# Modbus RTU Core and RS485 Layer

Chip-neutral Modbus RTU (slave, master and a polling scheduler) and the RS485
physical-layer interface it runs on. Shared by the encoder boards (RP2350, slave)
and the SBB (ESP32-S3, master). The two libraries live side by side:

| Library       | Path                          | Role                                                 |
| ------------- | ----------------------------- | ---------------------------------------------------- |
| `rs485`       | `firmware/shared/rs485`       | `rs485::Port` interface and one backend per chip     |
| `modbus-core` | `firmware/shared/modbus-core` | Protocol, `Slave`, `Master`, `MBUS`, device profiles |

## 1. Architecture

```
  slave application            master application
  (encoder comms_task)         (bucket EncoderBus)
         |                              |
         |     modbus/profiles/*.hpp    |   register maps, decoders, Device and Request builders
         |                              |
    modbus::Slave        modbus::MBUS -> modbus::Master        modbus-core
         |                              |
         +---- rs485::Port    modbus::Clock ----+              platform interfaces
                    |
       Rp2350Port  |  Esp32Port                                rs485 backends
```

Dependencies point downwards only. `modbus-core` never includes a profile or any
platform code, and `rs485` knows nothing about Modbus.

| Path                                             | Contents                                                        |
| ------------------------------------------------ | --------------------------------------------------------------- |
| `rs485/include/rs485/port.hpp`                   | `rs485::Port` interface                                         |
| `rs485/include/rs485/rp2350.hpp`, `src/rp2350.cpp`, `src/uart_tx.pio` | RP2350 backend                            |
| `rs485/include/rs485/esp32.hpp`, `src/esp32.cpp` | ESP32 backend                                                   |
| `modbus-core/include/modbus/`                    | `protocol.hpp` (CRC, frames), `slave.hpp`, `master.hpp`, `mbus.hpp`, `clock.hpp`, `heartbeat.hpp` |
| `modbus-core/include/modbus/profiles/`           | Per-device profiles (`encoder.hpp`)                             |
| `modbus-core/tests/`                             | Host unit tests                                                 |
| `firmware/components/rs485`, `modbus-core`       | ESP-IDF component wrappers, plus `modbus/esp32_clock.hpp`       |

## 2. Platform Interfaces

### `rs485::Port`

| Method                                                    | Contract                                                                                                                       |
| --------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| `send(data, len)`                                         | Blocks until the last stop bit is on the wire and the transceiver is back in receive mode. Returns false on failure.           |
| `receive(buf, cap, first_byte_timeout_us, frame_gap_us)`  | Waits up to `first_byte_timeout_us` for a byte, then reads until `frame_gap_us` of silence or `cap` bytes. 0 means nothing arrived. |
| `flush_rx()`                                              | Discards buffered input. The master calls it before every request so a late reply is never mistaken for the current one.       |
| `baud_hz()`                                               | Line rate, used to derive the frame gap.                                                                                       |

### `modbus::Clock`

`now_ms()`, `now_us()` (wraps; use differences only) and `delay_us()`. Only
`Master` and `MBUS` need one; a slave does not.

### Backends

| Backend              | Construction                                                       | Behaviour                                                                                                                                           |
| -------------------- | ------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| `rs485::Rp2350Port`  | `(uart, rx_pin, dir_pin, pio, sm, tx_pin, baud)`, then `init()`    | UART RX, PIO TX (`uart_tx.pio`), GPIO for DE/~RE. `receive()` busy-waits and rounds both timeouts up to whole milliseconds.                         |
| `rs485::Esp32Port`   | `(uart_port, tx, rx, dir, baud)`, then `init()` (returns bool)     | One UART for both directions. DIR is the UART's RTS in RS485 half-duplex mode. `receive()` sleeps until the first byte; the gap is timed in microseconds. Use UART1 or UART2. |
| `modbus::Esp32Clock` | default constructed                                                | `esp_timer` based. Lives in `firmware/components/modbus-core/include/modbus/esp32_clock.hpp`.                                                       |

A new chip needs the four `Port` methods, and a `Clock` if it will act as a master.
Both ends of a bus must use the same baud rate.

## 3. Protocol Summary

- **Frame.** `address, function, data, CRC16 (low byte first)`. A frame ends after
  3.5 character times of silence (11 bits per character):
  `frame_gap_us(baud)` = ceil(38 500 000 / baud) microseconds, 335 at 115200 baud.
- **Functions.** The slave implements 0x03 (read), 0x06 (write single) and 0x10
  (write multiple). The master issues 0x03 and 0x06.
- **Limits.** `kMaxFrameLen` is 64 bytes. A slave serves up to 29 registers per
  read; the master reads up to `kMaxResponseRegs` (16).
- **Broadcast.** Address 0 is executed by every slave and never answered, not even
  with an exception. After a broadcast the master waits out the frame gap itself.
- **Slave errors.** A bad CRC, a short frame or another slave's address gets no
  reply. An unknown function returns exception 0x01. Anything else a slave rejects
  (unreadable or unwritable register, malformed request) returns 0x02.
- **Example.** A complete angle read, byte by byte, is in section 8.

## 4. Slave

```cpp
modbus::Slave slave(port, address);
slave.set_read_handler(&read_reg, ctx);    // bool read_reg(uint16_t addr, uint16_t* out, void* ctx)
slave.set_write_handler(&write_reg, ctx);  // bool write_reg(uint16_t addr, uint16_t value, void* ctx)
for (;;)
    slave.poll();
```

- The core knows no registers. A handler returns false to reject a register, which
  becomes exception 0x02. A read fails as a whole if any register is rejected. A 0x10
  write applies registers in order and stops at the first rejection, without rollback.
- `poll()` services at most one frame and waits about one frame gap when the bus is
  idle, so the loop paces itself.
- `counters()` reports `frames_handled`, `crc_errors`, `short_frames` and
  `exceptions_sent`.
- One task owns the `Slave`, and the handlers run in it. Exchange data with other
  tasks through queues.
- `HeartbeatTracker` is the slave-side "has the master gone quiet" timer. The write
  handler calls `note(now_ms)`; other code reads `alive(now_ms, timeout_ms)`.

## 5. Master and MBUS

### `modbus::Master`

One blocking transaction per call. Each call flushes the receive buffer, sends,
waits for the reply, then waits out the frame gap before returning.

```cpp
modbus::Master master(port, clock, 50 /* reply timeout, ms */);
modbus::Response r = master.read_holding(slave, start_reg, count);
modbus::Response w = master.write_single(slave, reg, value);   // slave 0 = broadcast
```

| `Result`         | Meaning                                              | Counts as a link failure |
| ---------------- | ---------------------------------------------------- | ------------------------ |
| `Ok`             | Valid reply; `r.regs[]` and `r.reg_count` are filled | No                       |
| `ExceptionReply` | The slave answered with an exception (`exception_code`) | No: the board is alive |
| `Timeout`        | Nothing came back                                    | Yes                      |
| `CrcError`       | A reply arrived but was corrupt                      | Yes                      |
| `BadFrame`       | Valid CRC, wrong address, function or length         | Yes                      |
| `TransportError` | The port failed to send                              | Yes                      |
| `InvalidRequest` | Rejected locally (for example count 0); nothing sent | Ignored                  |

`Response` also carries `timestamp_ms` (when the transaction finished) and
`latency_us` (request start to reply).

### `modbus::MBUS<MaxDevices, MaxRequests>`

The master-side bus manager, the Modbus counterpart of Hi-CAN's `PacketManager`.
It owns one bus; one task calls `step()` in a loop. Capacity is fixed at compile
time and nothing allocates.

| Concept   | Type          | Meaning                                                                                              |
| --------- | ------------- | ---------------------------------------------------------------------------------------------------- |
| Device    | `Device`      | One slave: address, up to 2 poll jobs, a `HealthPolicy` and a state-change callback                  |
| Poll job  | `PollJob`     | Standing order: read `count` registers from `start_reg` every `period_ms`; `on_result` runs after every attempt |
| Request   | `Request`     | One-shot read or write (slave 0 = broadcast) with optional `retries` and an `on_done` callback       |
| Stats     | `DeviceStats` | Counters, consecutive failures, a 32-transaction history, latency and the health state               |

- **Scheduling.** `step()` runs at most one transaction and returns false when
  nothing is due; the caller then sleeps (`ms_until_next_due()` gives the wait).
  The job that has been due longest goes first. At most one request runs between
  polls, so a burst of commands cannot starve polling. A job that falls behind
  polls again immediately but does not burst to catch up.
- **Retries.** A request retries on `Timeout`, `CrcError`, `BadFrame` and
  `TransportError`, never on `Ok` or `ExceptionReply`. Poll jobs do not retry; the
  next period is the retry.
- **Sizing.** A two-register read is 17 bytes on the wire (11 bits each), about
  1.6 ms at 115200 baud. Add the frame gaps and the slave's gap detection when
  choosing poll periods.

**Health.** Each device is `Unknown`, `Ok`, `Degraded` (flaky) or `Lost` (sustained
silence). Transitions with the `HealthPolicy` defaults:

| Transition             | Condition                                                                                          |
| ---------------------- | -------------------------------------------------------------------------------------------------- |
| `Unknown` to `Ok`      | First good reply                                                                                   |
| to `Degraded`          | 2 consecutive failures, or an error rate of 20% or more over the last 32 (after 16 samples)        |
| to `Lost`              | 10 consecutive failures                                                                            |
| `Lost` to `Degraded`   | First good reply                                                                                   |
| `Degraded` to `Ok`     | 10 consecutive good replies, error rate below the threshold, and not latched                       |

With `latch_degraded` set, a `Degraded` device stays there until `clear_latch()`.

**Threading.** `MBUS` is not thread safe: only the task that calls `step()` may
call any of its methods. `Request` is trivially copyable so other tasks can hand
it over through a queue. Callbacks run inside `step()`, so keep them short. The
state-change callback fires before the `on_result` of the same transaction. Copy
`get_stats()` results out and publish them under a mutex.

## 6. Device Profiles

A profile is a header in `modbus/profiles/` that adds device knowledge on top of
the core and includes nothing else. `encoder.hpp` provides:

- the register enum and status bits (the single source of truth for the map);
- `decode_angle()` and `decode_status()`;
- `make_device(DeviceConfig)`, which builds a `Device` with two poll jobs (angle and
  status), because the write-only registers 2 and 3 break a block read;
- `zero_request()`, `discovery_request()` and `heartbeat_request()` (a broadcast).

Supporting another device means adding another profile header. `Slave`, `Master`
and `MBUS` do not change.

## 7. Build and Test

Pico SDK project (the encoder). `rs485_rp2350` exists only after `pico_sdk_init()`.

```cmake
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../shared/rs485 ${CMAKE_BINARY_DIR}/rs485)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../shared/modbus-core ${CMAKE_BINARY_DIR}/modbus-core)
target_link_libraries(app rs485_rp2350 modbus_core)
```

ESP-IDF project (the bucket). The wrappers in `firmware/components` compile the
shared sources; the application component requires them:

```cmake
idf_component_register(SRCS ${app_sources} REQUIRES modbus-core rs485)
```

Host tests, run from `firmware/shared/modbus-core`:

```
cmake -S . -B build -DMODBUS_CORE_BUILD_TESTS=ON && cmake --build build && ./build/modbus_core_tests
```

The tests connect a real `Master` to real `Slave`s over an in-memory bus with a fake
clock and fault injection (corrupt, dropped, disconnected), and double as usage
examples.

Constraints on the core: C++17; no heap, no exceptions and no RTTI (fixed buffers
and result codes); warning-free with `-Wall -Wextra` on GCC and Clang.

## 8. Examples

### On the wire: reading the angle

Slave address 1, encoder at 2048 counts (180.0 degrees), 115200 baud. The bytes are
asserted by `test_documented_angle_frames` in `tests/`, so they cannot drift from the
implementation.

| Frame               | Bytes                        | Meaning                                                                                                                     |
| ------------------- | ---------------------------- | --------------------------------------------------------------------------------------------------------------------------- |
| Request (8 bytes)   | `01 03 00 00 00 02 C4 0B`    | address 1, function 0x03 (read), start register `0000`, count `0002`, CRC                                                   |
| Reply (9 bytes)     | `01 03 04 08 00 07 08 FB A5` | address 1, function 0x03, 4 data bytes, register 0 = `0800` (2048 counts), register 1 = `0708` (1800 = 180.0 degrees), CRC  |
| Exception (5 bytes) | `01 83 02 C0 F1`             | function 0x83 (0x03 with the high bit set), code 0x02: the angle is unavailable (no magnet) or the register is rejected, CRC |

Registers are big-endian and the CRC is sent low byte first. The 17 bytes of a
successful read take about 1.6 ms of wire time at 115200 baud; the frame gaps and the
slave's gap detection add to that.

The master call that produces this exchange:

```cpp
modbus::Response r = master.read_holding(1, enc::kRegAngleRaw, 2);
enc::Angle a;
if (enc::decode_angle(r, &a))
    use(a.raw_counts, a.degrees_x10);  // 2048 and 1800
// otherwise r.result is ExceptionReply (exception_code 0x02), Timeout, CrcError, ...
```

In the bucket the master never makes this call directly. `MBUS` issues it as a poll
job, and the result travels:

```
MBUS::step() -> Master::read_holding() -> Port::send() / receive()
  -> Response -> on_angle() -> decode_angle() -> EncoderReading (under a mutex)
  -> control code: encoder_bus().get(EncoderId::LiftLeft, &reading)
```

On the encoder board, `Slave::poll()` receives the frame, checks the CRC and address,
and calls `read_register` for registers 0 and 1. The handler returns the latest sample
that `encoder_task` published at 1 kHz, so the angle is the last published sample, not a
live sensor read.

### Master: polling an encoder

Abridged from `firmware/excavation-bucket/src/encoder_bus.cpp`, which runs two
buses and six encoders. `publish` and `log_link_change` stand in for application code.

```cpp
#include "board_support.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "modbus/esp32_clock.hpp"
#include "modbus/mbus.hpp"
#include "modbus/profiles/encoder.hpp"
#include "rs485/esp32.hpp"

namespace enc = modbus::profiles::encoder;

static rs485::Esp32Port port(UART_NUM_1, bsp::RS485_1.tx, bsp::RS485_1.rx, bsp::RS485_1.dir, 115200);
static modbus::Esp32Clock clk;
static modbus::MBUS<3, 8> bus(port, clk, 50);  // 3 devices, 8 queued requests, 50 ms reply timeout
static QueueHandle_t commands;                 // other tasks post modbus::Request here

static void on_angle(const modbus::Response& r, void*)
{
    enc::Angle angle;
    if (enc::decode_angle(r, &angle))
        publish(angle.raw_counts);  // for example, copy into a mutex-protected table
}

static void on_link(uint8_t slave, modbus::DeviceState from, modbus::DeviceState to, void*)
{
    log_link_change(slave, from, to);  // Ok, Degraded or Lost
}

void bus_task(void*)
{
    if (!port.init())
        vTaskDelete(nullptr);

    enc::DeviceConfig cfg;
    cfg.slave = 1;  // the board's DIP value + 1
    cfg.angle_period_ms = 20;
    cfg.on_angle = &on_angle;
    cfg.on_state_change = &on_link;
    bus.add_device(enc::make_device(cfg));

    uint32_t last_heartbeat = clk.now_ms();
    for (;;)
    {
        // Only this task touches `bus`; commands arrive through the queue.
        modbus::Request req;
        while (xQueueReceive(commands, &req, 0) == pdTRUE)
            bus.submit(req);  // for example, enc::zero_request(1)

        if (clk.now_ms() - last_heartbeat >= 1000 && bus.submit(enc::heartbeat_request()))
            last_heartbeat = clk.now_ms();

        if (!bus.step())
            vTaskDelay(1);  // nothing due yet
    }
}
```

### Slave: how the encoder board sets it up

Abridged from `firmware/encoder-board/encoder/rtos/comms_task.cpp`. The application
supplies the register handlers; the core supplies framing, CRC, addressing and
exceptions.

```cpp
#include "modbus/heartbeat.hpp"
#include "modbus/profiles/encoder.hpp"
#include "modbus/slave.hpp"
#include "rs485/rp2350.hpp"
#include "shared_state.hpp"

using namespace modbus::profiles::encoder;

struct CommsContext
{
    SharedState* shared;  // queues owned by main()
    modbus::HeartbeatTracker heartbeat;
};

// Called once per register of a read request. Returning false becomes exception 0x02.
bool read_register(uint16_t address, uint16_t* out, void* context)
{
    auto* shared = static_cast<CommsContext*>(context)->shared;
    switch (address)
    {
    case kRegAngleRaw:
    case kRegAngleDegreesX10:
    {
        EncoderSample sample;
        if (xQueuePeek(shared->latest_sample, &sample, 0) != pdTRUE || !sample.valid)
            return false;  // no magnet, or the I2C read failed
        *out = (address == kRegAngleRaw) ? sample.reading.raw_counts
                                         : static_cast<uint16_t>(sample.reading.degrees * 10.0f);
        return true;
    }
    // kRegStatus and kRegDiscovery follow the same pattern.
    default:
        return false;  // write-only or unmapped register
    }
}

bool write_register(uint16_t address, uint16_t value, void* context)
{
    auto* comms = static_cast<CommsContext*>(context);
    switch (address)
    {
    case kRegHeartbeat:
        comms->heartbeat.note(to_ms_since_boot(get_absolute_time()));
        return true;
    case kRegDiscovery:
    {
        bool active = (value != 0);
        xQueueOverwrite(comms->shared->discovery_active, &active);  // read by led_task
        return true;
    }
    // kRegZeroCommand posts to a queue read by encoder_task.
    default:
        return false;  // read-only or unmapped register
    }
}

void comms_task(void* parameter)
{
    auto* shared = static_cast<SharedState*>(parameter);

    rs485::Rp2350Port port(uart1, RS485_RX_PIN, RS485_DIR_PIN, pio1, 0, RS485_TX_PIN, MODBUS_BAUD_HZ);
    port.init();

    CommsContext comms{shared, {}};
    modbus::Slave slave(port, static_cast<uint8_t>(shared->device_id + 1));  // address 0 is broadcast
    slave.set_read_handler(&read_register, &comms);
    slave.set_write_handler(&write_register, &comms);

    for (;;)
    {
        slave.poll();  // waits about one frame gap when idle: this is the task's pacing

        // Snapshot for led_task, so the board still shows "lost" if the master goes quiet.
        HeartbeatState hb;
        hb.heartbeat_ever_seen = comms.heartbeat.ever_seen();
        hb.master_alive = comms.heartbeat.alive(to_ms_since_boot(get_absolute_time()), kHeartbeatTimeoutMs);
        hb.last_heartbeat_ms = comms.heartbeat.last_ms();
        xQueueOverwrite(shared->heartbeat_state, &hb);
    }
}
```
