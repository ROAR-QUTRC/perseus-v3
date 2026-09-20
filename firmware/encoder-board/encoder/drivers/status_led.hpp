// status_led.hpp
//
// Computes what color the board's WS2812 status LED should show. Driving
// the LED itself (WS2812 PIO output) isn't implemented here -- this only
// decides the color; the LED task calls tick() regularly and pushes the
// result out.
//
// Visual language, highest priority first (only one shows at a time):
//   - Discovery mode active:        solid bright white (255,255,255) --
//                                    deliberately NOT dimmed like the rest,
//                                    it's a manual "which board is this"
//                                    beacon, meant to be unmissable.
//   - Every kHeartbeatBlipEveryNth-th
//     heartbeat to arrive
//     (within kHeartbeatBlipDurationMs): brief dim cyan blip, interrupts
//                                    whatever else was showing, then
//                                    reverts to it. Every single heartbeat
//                                    was too frequent/distracting in
//                                    practice, hence "every Nth" rather
//                                    than "every one."
//   - Never seen a heartbeat yet:   dim blue, blink at the heartbeat rate,
//                                    plus a quick triple-flash every
//                                    kWaitingTripleFlashIntervalMs (10s)
//                                    so a board sitting untouched for a
//                                    while still visibly confirms it's
//                                    running, not just slow-blinking.
//   - Heartbeat previously seen,
//     now lost:                     dim red, blink at the heartbeat rate.
//   - Magnet not detected:          dim amber, blink at the heartbeat rate.
//   - Magnet present, velocity +:   dim green, rapid flash
//                                    (kPositiveVelocityMultiplier x heartbeat rate)
//   - Magnet present, velocity -:   dim green, flash at 2x heartbeat rate
//   - Magnet present, no motion:    off. Green is motion-only now -- there
//                                    is no more "alive and idle" color;
//                                    the heartbeat blip covers that job.
//
// Colors are kept dim (matching the 0,0,10-style values used for the
// startup blink) since WS2812s at full brightness are painful to look at.
// Discovery white is the one deliberate exception -- see above.
//
// magnet_detected and velocity_sign can never disagree in a way that
// matters here: encoder_task only computes a nonzero velocity_sign when
// the magnet is present (see encoder_task.cpp), so "magnet missing" and
// "velocity" states are mutually exclusive by construction, not just by
// priority order.

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

    // Feed from ModbusRtu (via HeartbeatState): heartbeat_ever_seen
    // distinguishes "still waiting for the first heartbeat" (blue) from
    // "was alive, now timed out" (red) -- both are "not alive right now"
    // but should look different. last_heartbeat_ms anchors both the
    // lost/waiting blink phase and the heartbeat-blip window.
    void update_heartbeat_state(bool heartbeat_ever_seen, bool master_alive,
                                uint32_t last_heartbeat_ms);

    // Sign of the AS5600 angular velocity; 0 = not moving (or below
    // whatever noise-floor threshold the caller applies).
    void update_velocity(int velocity_sign);

    // Whether the AS5600 currently reports a magnet in range.
    void update_magnet_detected(bool magnet_detected);

    // Manual identify-this-board mode, driven by the Modbus discovery
    // register (kRegDiscovery) -- see modbus_rtu.hpp.
    void update_discovery(bool active);

    // Call regularly (e.g. every 10-20ms) from the LED task with a
    // monotonic ms clock (e.g. to_ms_since_boot(get_absolute_time())).
    // Returns the color the LED should show right now.
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
    // Low green and noticeably brighter than the other dim colors, on
    // purpose: the eye is disproportionately sensitive to green even at
    // low brightness, so any real green component here perceptually
    // washes this out toward pale yellow-white rather than reading as
    // amber. First attempt (10,4,0) still read as white on real
    // hardware; (10,2,0) also still did. This pushes both the ratio
    // (10:1 instead of 5:1) and the absolute level further.
    static constexpr Rgb kColorMagnetMissing{20, 2, 0};   // dim amber
    static constexpr Rgb kColorHeartbeatBlip{0, 10, 10};  // dim cyan
    static constexpr Rgb kColorDiscovery{255, 255, 255};  // full brightness, deliberately not dimmed

    static constexpr uint32_t kPositiveVelocityMultiplier = 4;  // "rapid" flash
    static constexpr uint32_t kNegativeVelocityMultiplier = 2;  // "double speed", per spec

    static constexpr uint32_t kHeartbeatBlipDurationMs = 100;
    static constexpr uint32_t kHeartbeatBlipEveryNth = 10;  // every heartbeat was too frequent

    static constexpr uint32_t kWaitingTripleFlashIntervalMs = 10000;  // every 10s
    static constexpr uint32_t kWaitingTripleFlashPulseMs = 100;       // each on/off segment
    static constexpr uint32_t kWaitingTripleFlashTotalMs =
        6 * kWaitingTripleFlashPulseMs;  // 3 on + 3 off = 600ms

    uint32_t heartbeat_period_ms_;

    bool heartbeat_ever_seen_ = false;
    bool master_alive_ = false;
    uint32_t last_heartbeat_ms_ = 0;
    uint32_t heartbeat_count_ = 0;  // counts distinct heartbeats seen, for the every-Nth blip
    int velocity_sign_ = 0;
    bool magnet_detected_ = true;  // optimistic default until the first sample arrives
    bool discovery_active_ = false;
};
