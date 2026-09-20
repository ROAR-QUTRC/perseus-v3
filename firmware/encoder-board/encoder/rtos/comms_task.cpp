// comms_task.cpp
// TODO: Potentially add dropped or corrupt MODBUS frame count, and other errors to status register
// TODO: Make some mention of corrupt frames, modbus master will handle this but just for debugging or status might be useful

#include "comms_task.hpp"

#include <cstdio>

#include "FreeRTOS.h"
#include "hardware/pio.h"
#include "modbus/heartbeat.hpp"
#include "modbus/profiles/encoder.hpp"
#include "modbus/slave.hpp"
#include "pico/time.h"
#include "rs485/rp2350.hpp"
#include "shared_state.hpp"
#include "task.h"

// RS485 pins; see rs485/rp2350.hpp for why TX is a PIO program.
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
#define MODBUS_BAUD_HZ 115200
#endif

namespace
{
    using namespace modbus::profiles::encoder;

    struct CommsContext
    {
        SharedState* shared;
        modbus::HeartbeatTracker heartbeat;
    };

    bool modbus_read_register(uint16_t address, uint16_t* out_value, void* context)
    {
        auto* shared = static_cast<CommsContext*>(context)->shared;

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
                status |= kStatusMagnetDetected;
            if (have_hb && hb.master_alive)
                status |= kStatusMasterAlive;
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
        auto* comms = static_cast<CommsContext*>(context);
        SharedState* shared = comms->shared;

        switch (address)
        {
        case kRegHeartbeat:
            // Any value counts. Heartbeat is this board's convention, so it is
            // handled here rather than in the modbus core.
            comms->heartbeat.note(to_ms_since_boot(get_absolute_time()));
            return true;
        case kRegZeroCommand:
        {
            if (value == 0)
                return true;  // 0 is a deliberate no-op, not an error
            EncoderCommand cmd = EncoderCommand::kZero;
            // Never block comms on encoder_task: if the queue is full, dropping a redundant zero is fine.
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

    // pio1: pio0 belongs to the status LED.
    rs485::Rp2350Port port(uart1, RS485_RX_PIN, RS485_DIR_PIN, pio1, 0, RS485_TX_PIN, MODBUS_BAUD_HZ);
    port.init();

    CommsContext comms{shared, {}};

    // Slave address = DIP ID + 1: address 0 is broadcast, so ID 0 still needs
    // a real address. Applied here, not in read_board_id().
    modbus::Slave modbus(port, static_cast<uint8_t>(shared->device_id + 1));
    modbus.set_read_handler(&modbus_read_register, &comms);
    modbus.set_write_handler(&modbus_write_register, &comms);

    printf("modbus: slave address %u, %u baud\n", shared->device_id + 1, MODBUS_BAUD_HZ);

    for (;;)
    {
        // Waits up to ~3.5 character times for a frame; that is this task's
        // pacing. It busy-waits, so led_task on this core must outrank it
        // (see led_task.hpp).
        modbus.poll();

        // Snapshot for led_task, so the board still shows "lost" if the master
        // stops addressing it.
        HeartbeatState hb;
        hb.heartbeat_ever_seen = comms.heartbeat.ever_seen();
        hb.master_alive = comms.heartbeat.alive(to_ms_since_boot(get_absolute_time()), kHeartbeatTimeoutMs);
        hb.last_heartbeat_ms = comms.heartbeat.last_ms();
        xQueueOverwrite(shared->heartbeat_state, &hb);
    }
}
