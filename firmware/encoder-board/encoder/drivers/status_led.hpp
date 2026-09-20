// status_led.hpp
//
// Decides the WS2812 status LED colour; the LED task calls tick() and pushes
// the result out. Colour meanings and priority order are in docs/modbus.md.
//
// Colours are dim on purpose (full-brightness WS2812s are painful to look at);
// discovery white is the one exception.

#pragma once

#include <cstdint>

class StatusLed
{
public:
    struct Rgb
    {
        uint8_t r, g, b;
    };

    explicit StatusLed(uint32_t heartbeat_period_ms);

    // heartbeat_ever_seen separates "waiting for the first heartbeat" (blue)
    // from "was alive, now timed out" (red). last_heartbeat_ms anchors the
    // blink phase and the heartbeat blip.
    void update_heartbeat_state(bool heartbeat_ever_seen, bool master_alive,
                                uint32_t last_heartbeat_ms);

    // Sign of the AS5600 angular velocity; 0 = not moving.
    void update_velocity(int velocity_sign);

    void update_magnet_detected(bool magnet_detected);

    // Identify-this-board mode, set through the Modbus discovery register.
    void update_discovery(bool active);

    // Call every 10-20ms with a monotonic ms clock; returns the colour to show now.
    Rgb tick(uint32_t now_ms) const;

private:
    enum class Mode
    {
        kDiscovery,
        kHeartbeatBlip,
        kWaitingForHeartbeat,
        kHeartbeatLost,
        kMagnetMissing,
        kVelocityPositive,
        kVelocityNegative,
        kIdle,
    };

    Mode current_mode(uint32_t now_ms) const;
    static bool blink_on(uint32_t now_ms, uint32_t period_ms);
    bool waiting_pattern_on(uint32_t now_ms) const;  // blue blink + 10s triple-flash
    bool heartbeat_blip_active(uint32_t now_ms) const;

    static constexpr Rgb kColorOff{0, 0, 0};
    static constexpr Rgb kColorStartup{0, 0, 10};  // dim blue
    static constexpr Rgb kColorOk{0, 10, 0};       // dim green
    static constexpr Rgb kColorLost{10, 0, 0};     // dim red
    // Deliberately much redder than the other dim colours: any real green
    // component washes amber out to pale white at low brightness.
    static constexpr Rgb kColorMagnetMissing{20, 2, 0};   // dim amber
    static constexpr Rgb kColorHeartbeatBlip{0, 10, 10};  // dim cyan
    static constexpr Rgb kColorDiscovery{255, 255, 255};  // full brightness, deliberately not dimmed

    static constexpr uint32_t kPositiveVelocityMultiplier = 4;  // "rapid" flash
    static constexpr uint32_t kNegativeVelocityMultiplier = 2;

    static constexpr uint32_t kHeartbeatBlipDurationMs = 100;
    static constexpr uint32_t kHeartbeatBlipEveryNth = 10;  // every heartbeat was too frequent

    static constexpr uint32_t kWaitingTripleFlashIntervalMs = 10000;
    static constexpr uint32_t kWaitingTripleFlashPulseMs = 100;  // each on/off segment
    static constexpr uint32_t kWaitingTripleFlashTotalMs =
        6 * kWaitingTripleFlashPulseMs;  // 3 on + 3 off

    uint32_t heartbeat_period_ms_;

    bool heartbeat_ever_seen_ = false;
    bool master_alive_ = false;
    uint32_t last_heartbeat_ms_ = 0;
    uint32_t heartbeat_count_ = 0;  // distinct heartbeats seen, for the every-Nth blip
    int velocity_sign_ = 0;
    bool magnet_detected_ = true;  // optimistic default until the first sample arrives
    bool discovery_active_ = false;
};
