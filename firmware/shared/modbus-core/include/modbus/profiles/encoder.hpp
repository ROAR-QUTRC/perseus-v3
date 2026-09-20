#pragma once

#include <cstdint>

#include "modbus/master.hpp"
#include "modbus/mbus.hpp"

// Encoder-board profile: its register map plus small helpers for driving it
// through the generic core. This layer depends on the core; the core never
// includes anything from profiles/. This is the single source of truth for the
// encoder board's register map; the Python tools in encoder-board/encoder/tools mirror it by hand.
namespace modbus::profiles::encoder
{
    enum Register : uint16_t
    {
        kRegAngleRaw = 0,         // R:  AS5600 counts 0-4095, after zero offset
        kRegAngleDegreesX10 = 1,  // R:  degrees * 10
        kRegZeroCommand = 2,      // W:  nonzero zeroes the current angle
        kRegHeartbeat = 3,        // W:  refreshes the board's master-alive timer
        kRegStatus = 4,           // R:  bitfield, see below
        kRegDiscovery = 5,        // RW: nonzero = solid white identify beacon
    };

    inline constexpr uint16_t kStatusMagnetDetected = 1u << 0;
    inline constexpr uint16_t kStatusMasterAlive = 1u << 1;

    struct Angle
    {
        uint16_t raw_counts = 0;
        uint16_t degrees_x10 = 0;
    };

    struct Status
    {
        bool magnet_detected = false;
        bool master_alive = false;
    };

    // Both return false unless the response is a successful read of the expected shape.
    inline bool decode_angle(const Response& r, Angle* out)
    {
        if (r.result != Result::Ok || r.reg_count != 2)
            return false;
        out->raw_counts = r.regs[0];
        out->degrees_x10 = r.regs[1];
        return true;
    }

    inline bool decode_status(const Response& r, Status* out)
    {
        if (r.result != Result::Ok || r.reg_count != 1)
            return false;
        out->magnet_detected = (r.regs[0] & kStatusMagnetDetected) != 0;
        out->master_alive = (r.regs[0] & kStatusMasterAlive) != 0;
        return true;
    }

    struct DeviceConfig
    {
        uint8_t slave = 0;
        uint32_t angle_period_ms = 20;
        uint32_t status_period_ms = 200;  // 0 = don't poll status
        ResultFn on_angle = nullptr;
        ResultFn on_status = nullptr;
        StateChangeFn on_state_change = nullptr;
        void* ctx = nullptr;
        HealthPolicy policy;
    };

    // The registers are not contiguous (2 and 3 are write-only), so angle and
    // status are two separate poll jobs.
    inline Device make_device(const DeviceConfig& config)
    {
        Device d;
        d.slave = config.slave;
        d.policy = config.policy;
        d.on_state_change = config.on_state_change;
        d.ctx = config.ctx;

        PollJob angle;
        angle.start_reg = kRegAngleRaw;
        angle.count = 2;
        angle.period_ms = config.angle_period_ms;
        angle.on_result = config.on_angle;
        angle.ctx = config.ctx;
        d.add_job(angle);

        if (config.status_period_ms > 0)
        {
            PollJob status;
            status.start_reg = kRegStatus;
            status.count = 1;
            status.period_ms = config.status_period_ms;
            status.on_result = config.on_status;
            status.ctx = config.ctx;
            d.add_job(status);
        }
        return d;
    }

    inline Request write_request(uint8_t slave, Register reg, uint16_t value, ResultFn on_done, void* ctx)
    {
        Request r;
        r.slave = slave;
        r.kind = Request::Kind::WriteReg;
        r.reg = reg;
        r.value_or_count = value;
        r.on_done = on_done;
        r.ctx = ctx;
        return r;
    }

    inline Request zero_request(uint8_t slave, ResultFn on_done = nullptr, void* ctx = nullptr)
    {
        return write_request(slave, kRegZeroCommand, 1, on_done, ctx);
    }

    inline Request discovery_request(uint8_t slave, bool on, ResultFn on_done = nullptr, void* ctx = nullptr)
    {
        return write_request(slave, kRegDiscovery, on ? 1 : 0, on_done, ctx);
    }

    // Broadcast: every board on the bus refreshes its master-alive timer, none reply.
    inline Request heartbeat_request(ResultFn on_done = nullptr, void* ctx = nullptr)
    {
        return write_request(kBroadcastAddress, kRegHeartbeat, 1, on_done, ctx);
    }
}  // namespace modbus::profiles::encoder
