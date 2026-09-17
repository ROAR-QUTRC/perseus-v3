// blink.cpp
//
// Blinks a single WS2812 ("NeoPixel", SparkFun COM-16347) connected to
// GPIO4, targeting bare RP2350A silicon (revision A2)
// Unlike a plaint LED, WS2812s are driven over a precisely-timed one-wire
// serial protocol, so this uses the RP2350's PIO peripheral 
// (see ws2812.pio) instead of gpio_put().

// pico/stdlib.h is the SDK's C API; its headers already wrap themselves in
// `extern "C"` guards, so they can be included directly from C++ with no 
// extra work.
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"   // generated from ws2812.pio by pico_generate_pio_header()

// GPIO wired to the NeoPixel's data-in pin. Overridable at build time, e.g.:
//   target_compile_definitions(blink PRIVATE NEOPIXEL_PIN=4)
#ifndef NEOPIXEL_PIN
#define NEOPIXEL_PIN 4
#endif

// Blink period in milliseconds. Overridable at build time without touching
// this file, e.g.g:
//   target_compile_definitions(blink PRIVATE LED_DELAY_MS=200)
#ifndef LED_DELAY_MS
#define LED_DELAY_MS 700
#endif

// Anonymous namespace - this file's private implementation detail (the C++
// equivalent of C's `static` for functions/variables at file scope). Nothing
// in here is visible to, or needs to be linked against, other source files.
namespace {
    constexpr float kWs2812FreqHz = 800000.0f; // standard WS2812 bit rate
    constexpr bool kIsRgbw = false; // COM-16347 is plain RGB, no white channel

    // WS2812s receive colour data as Green, Red, Blue -- not RGB order
    // data is packed into the the top 24 bits of the 32-bit word the PIO's
    // FIFO expects (hence the final <<8)
    void set_pixel(PIO pio, uint sm, uint8_t r, uint8_t g, uint8_t b)
    {
        uint32_t grb = (static_cast<uint32_t>(g) << 16) | 
                       (static_cast<uint32_t>(r) << 8) |
                       static_cast<uint32_t>(b);
                       pio_sm_put_blocking(pio, sm, grb << 8u);
    }

} // namespace

int main()
{
    PIO pio = pio0;
    uint sm = 0;

    // Load the PIO program once at startup and start a state machine
    // running it on NEOPIXEL_PIN
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, NEOPIXEL_PIN, kWs2812FreqHz, kIsRgbw);

    while (true) {
        set_pixel(pio, sm, 255, 255, 255); // white, full brightness = "on"
        sleep_ms(LED_DELAY_MS);
        set_pixel(pio, sm, 0, 0, 0);       // "off"
        sleep_ms(LED_DELAY_MS);
    }
}

