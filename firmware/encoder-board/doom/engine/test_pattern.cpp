// test_pattern.cpp
//
// Stand-in for the game while the backend is brought up: a scrolling pattern
// at Doom's 35 Hz tic rate, a square steered with the arrow keys, and Space
// switching to a red palette the way Doom flashes on damage.

#include <cstring>
#include <initializer_list>

#include "class/hid/hid.h"  // HID_KEY_* usage IDs
#include "engine.hpp"
#include "pico/stdlib.h"
#include "platform.hpp"

namespace
{
    constexpr int kWidth = platform::kScreenWidth;
    constexpr int kHeight = platform::kScreenHeight;
    constexpr uint32_t kTicUs = 1000000 / 35;
    constexpr int kSquareSize = 32;
    constexpr int kSquareSpeed = 3;  // pixels per tic
    constexpr uint8_t kSquareColour = 255;
    constexpr uint8_t kBarColour = 254;

    uint8_t frame[kWidth * kHeight];
    uint8_t normal_palette[256 * 3];
    uint8_t flash_palette[256 * 3];
    bool held[256];

    void build_palettes()
    {
        for (int i = 0; i < 256; ++i)
        {
            const uint8_t level = static_cast<uint8_t>(i * 2);  // pattern uses indices 0-127
            normal_palette[3 * i + 0] = level / 2;
            normal_palette[3 * i + 1] = level;
            normal_palette[3 * i + 2] = static_cast<uint8_t>(255 - level);
            flash_palette[3 * i + 0] = 255;
            flash_palette[3 * i + 1] = level / 4;
            flash_palette[3 * i + 2] = level / 4;
        }
        for (uint8_t* palette : {normal_palette, flash_palette})
        {
            std::memset(palette + 3 * kSquareColour, 255, 3);
            std::memset(palette + 3 * kBarColour, 200, 3);
        }
    }

    void read_keys()
    {
        platform::KeyEvent event;
        while (platform::input_poll(&event))
        {
            if (event.usage == 0)
                std::memset(held, 0, sizeof(held));
            else
                held[event.usage] = event.down;
        }
    }

    int clamp(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

void engine_run()
{
    build_palettes();
    int square_x = (kWidth - kSquareSize) / 2;
    int square_y = (kHeight - kSquareSize) / 2;
    absolute_time_t next_tic = get_absolute_time();

    for (uint32_t tic = 0;; ++tic)
    {
        read_keys();
        square_x += (held[HID_KEY_ARROW_RIGHT] - held[HID_KEY_ARROW_LEFT]) * kSquareSpeed;
        square_y += (held[HID_KEY_ARROW_DOWN] - held[HID_KEY_ARROW_UP]) * kSquareSpeed;
        square_x = clamp(square_x, 0, kWidth - kSquareSize);
        square_y = clamp(square_y, 0, kHeight - kSquareSize);

        for (int y = 0; y < kHeight; ++y)
            for (int x = 0; x < kWidth; ++x)
                frame[y * kWidth + x] = static_cast<uint8_t>((x + y + tic) & 0x7F);
        for (int y = 0; y < kSquareSize; ++y)
            std::memset(&frame[(square_y + y) * kWidth + square_x], kSquareColour, kSquareSize);
        std::memset(frame, kBarColour, tic % kWidth);  // top row: moves every tic, so a frozen stream is obvious

        platform::display_set_palette(held[HID_KEY_SPACE] ? flash_palette : normal_palette);
        platform::display_present(frame);

        next_tic = delayed_by_us(next_tic, kTicUs);
        sleep_until(next_tic);
    }
}
