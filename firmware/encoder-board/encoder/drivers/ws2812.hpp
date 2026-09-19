// ws2812.hpp
//
// Thin wrapper around the WS2812 PIO driver (ws2812.pio) for the board's
// status LED (GPIO4, "STATUS_LED" net).

#pragma once

#include <cstdint>

#include "hardware/pio.h"

class Ws2812
{
public:
    // pin: GPIO wired to the LED's data-in line.
    Ws2812(PIO pio, uint sm, uint pin);

    // Loads the PIO program and starts the state machine. Call once at boot.
    void init();

    // Callers pass plain RGB; WS2812s actually want GRB order, which is
    // handled internally.
    void set_pixel(uint8_t r, uint8_t g, uint8_t b);

private:
    PIO pio_;
    uint sm_;
    uint pin_;
};
