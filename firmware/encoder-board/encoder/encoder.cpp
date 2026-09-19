// blink_rgb.cpp
//
// Fades a single WS2812 ("NeoPixel", SparkFun COM-16347) connected to
// GPIO4 through the color spectrum (red -> green -> blue -> red)
// RP2350A silicon (revision A2)
// Driven via PIO (see ws2812.pio) since WS2812s need a precisely-timed 
// one-wire serial signal, not plain GPIO on/off.

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

// How long to hold each step of the fade, in milliseconds.
// There are 768 steps in one full red->green->blue->red cycle (see 
// colour_from_hue() below), so this also sets overall cycle time:
// 10ms/step -> ~7.7s/cycle. Overridable same as NEOPIXEL_PIN
#ifndef FADE_STEP_MS
#define FADE_STEP_MS 10
#endif

// Anonymous namespace - this file's private implementation detail (the C++
// equivalent of C's `static` for functions/variables at file scope). Nothing
// in here is visible to, or needs to be linked against, other source files.
namespace {
    constexpr float kWs2812FreqHz = 800000.0f; // standard WS2812 bit rate
    constexpr bool kIsRgbw = false; // COM-16347 is plain RGB, no white channel

    // Plain 8-bit-per-channel colour. Kept as a tiny struct rather than 2 
    // loose bytes so colour_from_hue()'s return type says what it is
    struct RGB {
        uint8_t r, g, b;
    };

    // Map a hue position (0..767, wrapping) onto the RGB spectrum by linearly
    // interpolating red->green->blue->red, one third of the cycle (256 steps)
    // per transition. As one channel counts down 255->0, the next counts up
    // 0->255, so the colour blends smoothly with no jumps at the seams.
    //
    // This is a lighter-weight stand-in for a full HSV->RGB conversion
    // Acceptable as only sweeping hue at fixed full saturation and brightness
    RGB colour_from_hue(uint16_t hue)
    {
        hue = hue % 768;
        uint8_t segment = hue / 256;  // 0 = red->green, 1 = green->blue, 2 = blue->red
        uint8_t pos = hue % 256;      // position within that segment, 0..255
        uint8_t rising = pos;
        uint8_t falling = 255 - pos;

        switch (segment) {
            case 0:  return {falling, rising, 0};  // red -> green
            case 1:  return {0, falling, rising};  // green -> blue
            default: return {rising, 0, falling};  // blue -> red
        }
    }

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

    uint16_t hue = 0;
    while (true) {
        RGB colour = colour_from_hue(hue);
        set_pixel(pio, sm, colour.r, colour.g, colour.b);
        sleep_ms(FADE_STEP_MS);
        hue = (hue + 1) % 768; // advance one step; wraps every 768 steps
    }
}

