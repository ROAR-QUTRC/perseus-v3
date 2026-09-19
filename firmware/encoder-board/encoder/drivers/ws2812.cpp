// ws2812.cpp

#include "ws2812.hpp"

#include "hardware/clocks.h"
#include "ws2812.pio.h"  // generated from ws2812.pio by pico_generate_pio_header()

namespace
{
    constexpr float kWs2812FreqHz = 800000.0f;  // standard WS2812 bit rate
    constexpr bool kIsRgbw = false;             // this board's LED is plain RGB
}  // namespace

Ws2812::Ws2812(PIO pio, uint sm, uint pin)
    : pio_(pio),
      sm_(sm),
      pin_(pin)
{
}

void Ws2812::init()
{
    uint offset = pio_add_program(pio_, &ws2812_program);
    ws2812_program_init(pio_, sm_, offset, pin_, kWs2812FreqHz, kIsRgbw);
}

void Ws2812::set_pixel(uint8_t r, uint8_t g, uint8_t b)
{
    // WS2812s receive colour data as Green, Red, Blue -- not RGB order.
    // Data is packed into the top 24 bits of the 32-bit word the PIO's
    // FIFO expects (hence the final <<8).
    uint32_t grb = (static_cast<uint32_t>(g) << 16) | (static_cast<uint32_t>(r) << 8) |
                   static_cast<uint32_t>(b);
    pio_sm_put_blocking(pio_, sm_, grb << 8u);
}
