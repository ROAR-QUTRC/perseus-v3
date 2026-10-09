// jaw_homing.hpp
//
// Homing for the jaws, the one joint whose zero is found by feel. They open
// until the actuators cut out on their end switches, then bite the frame and
// back off kBites times. The encoders are zeroed while clenched on the last
// bite, so 0 degrees is the clenched frame. It runs on current and time alone:
// it never reads the angle it is about to redefine, and the bank's angle
// limits don't apply to it.

#pragma once

#include <cstdint>
#include <optional>

#include "esp_log.h"

// Where a bank is in its homing routine; each run starts from a fresh one.
struct HomingState
{
    uint8_t phase = 0;
    uint32_t since = 0;                  // when the phase began
    std::optional<uint32_t> held_since;  // since when its exit condition has held, while it does
};

// What the routine wants this control tick.
struct HomingStep
{
    float duty = 0.0f;  // % for both actuators; positive opens (raises the angle)
    bool zero = false;  // zero the encoders now
    bool done = false;  // finished, or given up
};

// A bank with one of these in its config runs it on SET_ZERO_POS: once per
// control tick, with the bank's current in amps as GET_CURRENT reports it.
using HomingRoutine = HomingStep(HomingState& state, uint32_t now_ms, float amps);

inline HomingStep jaw_homing(HomingState& s, uint32_t now_ms, float amps)
{
    constexpr float kSpeed = 100.0f;         // % duty
    constexpr float kBiteCurrent = 1.5f;     // amps: at or above, the jaws are clenched on the frame
    constexpr float kIdleCurrent = 0.2f;     // amps: at or below, the actuators have cut out at the open end
    constexpr uint8_t kBites = 3;            // bite and back-off cycles; the zero is taken on the last
    constexpr uint32_t kBackoffMs = 500;     // opening after each bite
    constexpr uint32_t kZeroHoldMs = 300;    // last bite: clench held while the encoders take the zero
    constexpr uint32_t kInrushMs = 200;      // current ignored after each start or reversal
    constexpr uint32_t kDetectMs = 100;      // current must stay past its threshold this long
    constexpr uint32_t kTimeoutMs = 30'000;  // per open or bite
    static_assert(kIdleCurrent < kBiteCurrent);

    constexpr const char* TAG = "jaw_homing";
    const auto give_up = [](const char* why)
    {
        ESP_LOGW(TAG, "aborted, %s", why);
        return HomingStep{.done = true};
    };

    // Phase 0 opens to the end switches. After it, odd phases bite and even
    // ones back off; the last back-off first holds the clench for the zero.
    constexpr uint8_t kLast = 2 * kBites;
    const uint32_t elapsed = now_ms - s.since;
    const bool biting = s.phase & 1;
    const bool timed = s.phase && !biting;
    const bool stalled = amps >= kBiteCurrent;
    const auto holding = [&]
    { return s.phase == kLast && now_ms - s.since < kZeroHoldMs; };

    // A back-off ends on the clock. An open or a bite ends on the current:
    // idle or stalled, once the inrush has passed.
    const bool met = timed ? !holding() : elapsed >= kInrushMs && (stalled || (!biting && amps <= kIdleCurrent));
    if (!met)
        s.held_since.reset();
    else if (!s.held_since)
        s.held_since = now_ms;

    HomingStep step;
    if (s.held_since && now_ms - *s.held_since >= (timed ? kBackoffMs : kDetectMs))
    {
        // Stalling on the way open means the jaws ran into something, or are closing instead.
        if (!s.phase && stalled)
            return give_up("stalled while opening");
        if (s.phase == kLast)
        {
            ESP_LOGI(TAG, "done");
            return {.done = true};
        }
        if (biting)
            ESP_LOGI(TAG, "bite %d/%d at %.2f A", (s.phase + 1) / 2, kBites, amps);
        else if (!s.phase)
            ESP_LOGI(TAG, "open end reached (%.2f A)", amps);

        s = {static_cast<uint8_t>(s.phase + 1), now_ms, now_ms};
        step.zero = s.phase == kLast;
    }
    else if (!timed && elapsed >= kTimeoutMs)
        return give_up(biting ? "no current spike while closing" : "current never fell to idle while opening");

    step.duty = (s.phase & 1) || holding() ? -kSpeed : kSpeed;
    return step;
}
