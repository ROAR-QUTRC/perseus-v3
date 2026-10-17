// jaw_twist.hpp
//
// Patch for a hardware fault that can't be fixed yet: the jaws twist when they
// clamp shut, turning the encoder housing so it reads 10-20 degrees open while
// they are closed. A driven joint can only move with its drive, so a reading
// that climbs while the jaws are driven shut is the twist, not the jaws. The
// angle is then held at the lowest reading of that close, the contact angle
// (about 0 on sand or nothing), until the jaws are driven open and the reading,
// once the twist has unwound, rises with the drive again.

#pragma once

#include <algorithm>
#include <cstdint>
#include <optional>

// What an angle correction remembers from tick to tick.
struct AngleCorrectionState
{
    int8_t move = 0;            // the way the jaws are being moved: -1 lowering the angle, +1 raising it
    float low = 0.0f;           // the lowest reading since they started moving that way
    float last = 0.0f;          // the last reading, for while the encoder is stale
    std::optional<float> held;  // the angle used and reported instead of the reading
};

// A bank with one of these in its config passes it every tick's reading
// (nullopt while the encoder is stale) and the way that side has been driven
// since the last tick: -1 lowering the angle, +1 raising it, 0 stopped. It
// returns the angle to use instead of the reading, or nullopt to trust it.
using AngleCorrection = std::optional<float>(AngleCorrectionState& state, std::optional<float> reading, int8_t drive);

inline std::optional<float> jaw_twist(AngleCorrectionState& s, std::optional<float> reading, int8_t drive)
{
    constexpr float kRiseDegrees = 2.0f;  // rise above the low point: twist while closing, real motion while opening
    constexpr float kShutDegrees = 0.0f;  // the jaws can't close past this, so nothing below it is reported

    // A move lasts until the drive reverses, through pauses, but not through
    // being driven blind. An opening that couldn't be seen lets go, since
    // seeing it would have.
    if (!reading)
    {
        if (drive)
            s.move = 0;
        if (drive > 0)
            s.held.reset();
    }
    // Standing still says nothing either way about the twist.
    else if (s.last = *reading; drive)
    {
        if (drive != s.move)
            s.move = drive, s.low = *reading;  // each move starts from where the reading is
        s.low = std::min(s.low, *reading);
        // The twist only adds to the reading, so the jaws are never above it.
        if (s.held)
            s.held = std::min(*s.held, *reading);

        // Closing lowers the angle, so a rise while closing is the twist, and a
        // rise while opening is the jaws moving again.
        const bool risen = *reading > s.low + kRiseDegrees;
        if (risen && drive < 0 && !s.held)
            s.held = s.low;
        else if (risen && drive > 0)
            s.held.reset();
    }
    // Shut is as far as the jaws go: nothing below it, held or read, is passed on.
    if (s.held || s.last < kShutDegrees)
        return std::max(s.held.value_or(s.last), kShutDegrees);
    return std::nullopt;
}
