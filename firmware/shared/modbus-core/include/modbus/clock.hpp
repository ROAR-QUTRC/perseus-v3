#pragma once

#include <cstdint>

namespace modbus
{
    // The time the master side needs: timestamps for stats and scheduling, and
    // short waits between frames. Slaves don't need one.
    class Clock
    {
    public:
        virtual ~Clock() = default;

        virtual uint32_t now_ms() = 0;
        virtual uint32_t now_us() = 0;  // wraps; only ever used for differences
        virtual void delay_us(uint32_t us) = 0;
    };
}  // namespace modbus
