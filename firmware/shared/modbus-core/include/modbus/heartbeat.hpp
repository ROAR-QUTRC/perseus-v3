#pragma once

#include <cstdint>

namespace modbus
{
    // Slave-side "has the master gone quiet" bookkeeping. The application's
    // write handler calls note() when the heartbeat register is written.
    class HeartbeatTracker
    {
    public:
        void note(uint32_t now_ms)
        {
            last_ms_ = now_ms;
            seen_ = true;
        }

        bool alive(uint32_t now_ms, uint32_t timeout_ms) const { return seen_ && (now_ms - last_ms_) <= timeout_ms; }

        // Distinguishes "never got one" from "got one, then it timed out".
        bool ever_seen() const { return seen_; }
        uint32_t last_ms() const { return last_ms_; }

    private:
        uint32_t last_ms_ = 0;
        bool seen_ = false;
    };
}  // namespace modbus
