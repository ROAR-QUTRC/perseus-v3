// board_id.hpp
//
// Reads this board's 3-bit device ID (0-7) from its DIP switch.

#pragma once

#include <cstdint>

// GPIOs wired to the DIP switch (SW1), one bit each. Each pin is hard
// shorted to GND when its switch is on, and left floating otherwise --
// read with the RP2350's internal pull-up enabled. Switch on drives the
// pin low, and that's taken as the bit being set: on = 1, off (pulled
// high) = 0.
inline constexpr uint32_t kBoardIdPin0 = 11;  // ID_ADDR0
inline constexpr uint32_t kBoardIdPin1 = 10;  // ID_ADDR1
inline constexpr uint32_t kBoardIdPin2 = 9;   // ID_ADDR2

// Configures the DIP switch pins and returns the 3-bit device ID (0-7).
// Call once at boot.
uint8_t read_board_id();
