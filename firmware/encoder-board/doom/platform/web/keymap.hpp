// keymap.hpp
//
// Browser KeyboardEvent.code (e.g. "ArrowUp", "KeyW") to USB HID usage ID,
// the key code the rest of the firmware uses.

#pragma once

#include <cstddef>
#include <cstdint>

namespace keymap
{
    // 0 if the key isn't one the game can use.
    uint8_t usage_from_code(const char* code, size_t length);
}
