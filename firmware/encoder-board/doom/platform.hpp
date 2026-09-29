// platform.hpp
//
// What the game needs from the device it is shown on. One backend is built in,
// chosen by DOOM_PLATFORM: web (USB network link to a browser) now, hdmi TBD.

#pragma once

#include <cstdint>

namespace platform
{
    inline constexpr int kScreenWidth = 320;
    inline constexpr int kScreenHeight = 200;

    // Keys are USB HID usage IDs (what a USB keyboard sends), so every backend
    // feeds the game through one key table. Usage 0 with down == false means
    // "release every key" (e.g. the browser tab lost focus).
    struct KeyEvent
    {
        uint8_t usage;
        bool down;
    };

    // Call once on core 0 before anything else. The backend takes core 1, and
    // with it stdio: don't printf from core 0 in the web build.
    void init();

    // 256 RGB triplets. Applies to frames presented after this call.
    void display_set_palette(const uint8_t* rgb);

    // kScreenWidth x kScreenHeight palette indices. Never blocks: a frame that
    // arrives while the backend is still busy with the last one is dropped.
    void display_present(const uint8_t* pixels);

    bool input_poll(KeyEvent* event);
}  // namespace platform
