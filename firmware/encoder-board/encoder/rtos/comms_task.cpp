// comms_task.cpp
// TODO: Potentially add dropped or corrupt MODBUS frame count, and other errors to status register

#include "comms_task.hpp"

#include <cstdio>

#include "FreeRTOS.h"
#include "hardware/pio.h"
#include "modbus_rtu.hpp"
#include "pico/time.h"
#include "rs485_transport.hpp"
#include "shared_state.hpp"
#include "task.h"

// RS485/Modbus link -- see the class comment in rs485_transport.hpp for
// why TX is on a PIO program rather than the hardware UART.
#ifndef RS485_RX_PIN
#define RS485_RX_PIN 5
#endif

#ifndef RS485_DIR_PIN
#define RS485_DIR_PIN 6
#endif

#ifndef RS485_TX_PIN
#define RS485_TX_PIN 7
#endif

#ifndef MODBUS_BAUD_HZ
#define MODBUS_BAUD_HZ 9600
#endif

namespace
{
    bool modbus_read_register(uint16_t address, uint16_t* out_value, void* context)
    {
        auto* shared = static_cast<SharedState*>(context);

        switch (address)
        {
        case kRegAngleRaw:
        case kRegAngleDegreesX10:
        {
            EncoderSample sample;
            if (xQueuePeek(shared->latest_sample, &sample, 0) != pdTRUE || !sample.valid)
                return false;
            *out_value = (address == kRegAngleRaw)
                             ? sample.reading.raw_counts
                             : static_cast<uint16_t>(sample.reading.degrees * 10.0f);
            return true;
        }
        case kRegStatus:
        {
            EncoderSample sample;
            bool have_sample = (xQueuePeek(shared->latest_sample, &sample, 0) == pdTRUE);

            HeartbeatState hb;
            bool have_hb = (xQueuePeek(shared->heartbeat_state, &hb, 0) == pdTRUE);

            uint16_t status = 0;
            if (have_sample && sample.magnet_detected)
                status |= (1u << 0);
            if (have_hb && hb.master_alive)
                status |= (1u << 1);
            *out_value = status;
            return true;
        }
        case kRegDiscovery:
        {
            bool active = false;
            xQueuePeek(shared->discovery_active, &active, 0);
            *out_value = active ? 1 : 0;
            return true;
        }
        default:
            return false;  // write-only or unknown register -> illegal data address
        }
    }

    bool modbus_write_register(uint16_t address, uint16_t value, void* context)
    {
        auto* shared = static_cast<SharedState*>(context);

        switch (address)
        {
        case kRegZeroCommand:
        {
            if (value == 0)
                return true;  // 0 is a deliberate no-op, not an error
            EncoderCommand cmd = EncoderCommand::kZero;
            // Don't block the comms loop waiting on encoder_task; if
            // the (length-4) queue is ever full, encoder_task has
            // fallen badly behind and dropping a redundant zero
            // request is the right failure mode anyway.
            return xQueueSend(shared->encoder_commands, &cmd, 0) == pdTRUE;
        }
        case kRegDiscovery:
        {
            bool active = (value != 0);
            xQueueOverwrite(shared->discovery_active, &active);
            return true;
        }
        default:
            return false;  // read-only or unknown register -> illegal data address
        }
    }
}  // namespace

void comms_task(void* parameter)
{
    auto* shared = static_cast<SharedState*>(parameter);

    // pio1, not pio0 (which the status LED owns), so the two PIO users
    // don't compete for program space or state machines.
    Rs485Transport rs485(uart1, RS485_RX_PIN, RS485_DIR_PIN, pio1, 0, RS485_TX_PIN, MODBUS_BAUD_HZ);
    rs485.init();

    // Modbus address = DIP-switch device ID + 1 (1-8) -- address 0 is
    // reserved for broadcast, see the class comment in modbus_rtu.hpp.
    ModbusRtu modbus(&rs485, static_cast<uint8_t>(shared->device_id + 1));
    modbus.set_read_handler(&modbus_read_register, shared);
    modbus.set_write_handler(&modbus_write_register, shared);

    printf("modbus: slave address %u, %u baud\n", shared->device_id + 1, MODBUS_BAUD_HZ);

    for (;;)
    {
        // Blocks for up to ~3.5 character times if no frame is arriving --
        // see the note on Rs485Transport::receive(). That's this task's
        // whole pacing; no extra vTaskDelay() needed. It never blocks
        // cooperatively though (busy-waits instead), which is why
        // led_task on this same core needs to sit at a strictly higher
        // priority -- see led_task.hpp.
        modbus.poll();

        HeartbeatState hb;
        hb.heartbeat_ever_seen = modbus.heartbeat_ever_seen();
        hb.master_alive = modbus.master_alive(to_ms_since_boot(get_absolute_time()), kHeartbeatTimeoutMs);
        hb.last_heartbeat_ms = modbus.last_heartbeat_ms();
        xQueueOverwrite(shared->heartbeat_state, &hb);
    }
}
