// ws2812.hpp
//
// Wrapper around the WS2812 PIO driver (ws2812.pio) for the status LED (GPIO4).

#pragma once

#include <cstdint>

#include "hardware/pio.h"

class Ws2812
{
public:
    Ws2812(PIO pio, uint sm, uint pin);

    // Loads the PIO program and starts the state machine. Call once.
    void init();

    // Takes plain RGB; converted to the WS2812's GRB order internally.
    void set_pixel(uint8_t r, uint8_t g, uint8_t b);

private:
    PIO pio_;
    uint sm_;
    uint pin_;
};
