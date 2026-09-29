// png.hpp
//
// Palette PNG encoder for the web backend's frame stream. Plain C++, no Pico
// dependencies, so it can be tested on a PC.

#pragma once

#include <cstddef>
#include <cstdint>

namespace png
{
    // Input is PNG scanlines: per row, a filter byte (0 = none) then `width`
    // palette indices. display_present() writes frames in this layout directly.
    constexpr size_t scanlines_size(int width, int height)
    {
        return static_cast<size_t>(width + 1) * static_cast<size_t>(height);
    }

    // Worst case for a 320x200 frame: every byte a 9-bit literal, plus headers.
    inline constexpr size_t kMaxEncodedSize = 74 * 1024;

    // Encodes with a 256-entry RGB palette. Returns the PNG size in bytes, or 0
    // if `out_size` is too small or the image is over 65534 scanline bytes.
    size_t encode(const uint8_t* scanlines, int width, int height, const uint8_t* palette_rgb, uint8_t* out,
                  size_t out_size);
}  // namespace png
