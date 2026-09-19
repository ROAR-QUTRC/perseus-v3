// status_led.cpp

#include "status_led.hpp"

StatusLed::StatusLed(uint32_t heartbeat_period_ms) : heartbeat_period_ms_(heartbeat_period_ms) {}

void StatusLed::update_heartbeat_state(bool heartbeat_ever_seen, bool master_alive,
                                        uint32_t last_heartbeat_ms)
{
    // A genuinely new heartbeat landed (not just another tick reflecting
    // the same one) whenever the timestamp moves. Counted here, rather
    // than in ModbusRtu, so the every-Nth blip logic stays entirely
    // self-contained in this class.
    if (heartbeat_ever_seen && last_heartbeat_ms != last_heartbeat_ms_)
        ++heartbeat_count_;

    heartbeat_ever_seen_ = heartbeat_ever_seen;
    master_alive_ = master_alive;
    last_heartbeat_ms_ = last_heartbeat_ms;
}

void StatusLed::update_velocity(int velocity_sign) { velocity_sign_ = velocity_sign; }

void StatusLed::update_magnet_detected(bool magnet_detected) { magnet_detected_ = magnet_detected; }

void StatusLed::update_discovery(bool active) { discovery_active_ = active; }

StatusLed::Mode StatusLed::current_mode(uint32_t now_ms) const
{
    if (discovery_active_)
        return Mode::kDiscovery;

    if (heartbeat_blip_active(now_ms))
        return Mode::kHeartbeatBlip;

    if (!heartbeat_ever_seen_)
        return Mode::kWaitingForHeartbeat;
    if (!master_alive_)
        return Mode::kHeartbeatLost;
    if (!magnet_detected_)
        return Mode::kMagnetMissing;
    if (velocity_sign_ > 0)
        return Mode::kVelocityPositive;
    if (velocity_sign_ < 0)
        return Mode::kVelocityNegative;
    return Mode::kIdle;
}

bool StatusLed::blink_on(uint32_t now_ms, uint32_t period_ms)
{
    if (period_ms == 0)
        return false;
    return (now_ms % period_ms) < (period_ms / 2);
}

bool StatusLed::waiting_pattern_on(uint32_t now_ms) const
{
    uint32_t t_in_cycle = now_ms % kWaitingTripleFlashIntervalMs;
    if (t_in_cycle < kWaitingTripleFlashTotalMs)
    {
        // 3 quick pulses: on/off/on/off/on/off, kWaitingTripleFlashPulseMs each.
        uint32_t segment = t_in_cycle / kWaitingTripleFlashPulseMs;  // 0..5
        return (segment % 2) == 0;
    }
    return blink_on(now_ms, heartbeat_period_ms_);
}

bool StatusLed::heartbeat_blip_active(uint32_t now_ms) const
{
    if (!heartbeat_ever_seen_)
        return false;
    if (heartbeat_count_ == 0 || (heartbeat_count_ % kHeartbeatBlipEveryNth) != 0)
        return false;
    return (now_ms - last_heartbeat_ms_) < kHeartbeatBlipDurationMs;
}

StatusLed::Rgb StatusLed::tick(uint32_t now_ms) const
{
    switch (current_mode(now_ms))
    {
        case Mode::kDiscovery:
            return kColorDiscovery;
        case Mode::kHeartbeatBlip:
            return kColorHeartbeatBlip;
        case Mode::kWaitingForHeartbeat:
            return waiting_pattern_on(now_ms) ? kColorStartup : kColorOff;
        case Mode::kHeartbeatLost:
            return blink_on(now_ms, heartbeat_period_ms_) ? kColorLost : kColorOff;
        case Mode::kMagnetMissing:
            return blink_on(now_ms, heartbeat_period_ms_) ? kColorMagnetMissing : kColorOff;
        case Mode::kVelocityPositive:
            return blink_on(now_ms, heartbeat_period_ms_ / kPositiveVelocityMultiplier)
                       ? kColorOk
                       : kColorOff;
        case Mode::kVelocityNegative:
            return blink_on(now_ms, heartbeat_period_ms_ / kNegativeVelocityMultiplier)
                       ? kColorOk
                       : kColorOff;
        case Mode::kIdle:
            return kColorOff;
    }
    return kColorOff;
}
