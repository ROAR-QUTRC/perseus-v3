// backend.cpp
//
// Web backend: core 1 serves the game to a browser over the USB network link.
//
// Frames cross from core 0 to core 1 through one slot guarded by frame_ready:
// core 0 only writes the slot while it is false, core 1 only reads it while it
// is true. The atomic's release/acquire ordering makes the slot's contents
// visible to the other core before the flag flips. Keys go the other way
// through a pico_util queue, which is safe across cores.

#include <atomic>
#include <cstdio>
#include <cstring>

#include "http_server.hpp"
#include "keymap.hpp"
#include "net.hpp"
#include "pico/multicore.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "platform.hpp"
#include "png.hpp"
#include "tusb.h"

namespace
{
    constexpr int kWidth = platform::kScreenWidth;
    constexpr int kHeight = platform::kScreenHeight;
    constexpr size_t kScanlines = png::scanlines_size(kWidth, kHeight);
    constexpr size_t kPaletteBytes = 256 * 3;
    constexpr uint kKeyQueueLength = 32;
    constexpr uint32_t kStatsPeriodUs = 1000000;

    uint8_t current_palette[kPaletteBytes];  // core 0 only

    uint8_t slot_scanlines[kScanlines];  // the frame slot
    uint8_t slot_palette[kPaletteBytes];
    std::atomic<bool> frame_ready{false};
    std::atomic<uint32_t> frames_dropped{0};

    queue_t key_queue;

    uint8_t png_buffer[png::kMaxEncodedSize];  // core 1 only
    uint32_t core1_stack[4096];                // 16 KiB

    // Core 1 stats, printed once a second.
    uint32_t encode_us_total = 0;
    uint32_t png_bytes_total = 0;
    uint32_t frames_encoded = 0;
    uint32_t keys_received = 0;

    void on_key(const char* body, size_t length)
    {
        platform::KeyEvent event{};
        if (length == 1 && body[0] == 'R')
        {
            event = {0, false};
        }
        else if (length >= 2 && (body[0] == '0' || body[0] == '1'))
        {
            event.usage = keymap::usage_from_code(body + 1, length - 1);
            event.down = body[0] == '1';
            if (event.usage == 0)
                return;  // a key the game has no use for
        }
        else
        {
            return;
        }
        ++keys_received;
        queue_try_add(&key_queue, &event);  // full means core 0 has stalled; dropping is fine
    }

    void send_frame_if_ready()
    {
        if (!frame_ready.load(std::memory_order_acquire) || !http::stream_ready())
            return;

        const uint32_t start = time_us_32();
        const size_t size = png::encode(slot_scanlines, kWidth, kHeight, slot_palette, png_buffer, sizeof(png_buffer));
        frame_ready.store(false, std::memory_order_release);  // the slot is free again

        encode_us_total += time_us_32() - start;
        png_bytes_total += size;
        ++frames_encoded;
        if (size)
            http::stream_send(png_buffer, size);
    }

    void print_stats()
    {
        static uint32_t last_us = time_us_32();
        const uint32_t now = time_us_32();
        if (now - last_us < kStatsPeriodUs)
            return;
        last_us = now;

        const uint32_t sent = http::take_frames_sent();
        if (frames_encoded)
            printf("stream: %lu fps, %lu KB/s, png %lu B, encode %lu us, dropped %lu, keys %lu\n",
                   (unsigned long)sent, (unsigned long)(png_bytes_total / 1024), (unsigned long)(png_bytes_total / frames_encoded),
                   (unsigned long)(encode_us_total / frames_encoded), (unsigned long)frames_dropped.exchange(0),
                   (unsigned long)keys_received);
        encode_us_total = png_bytes_total = frames_encoded = keys_received = 0;
    }

    void core1_main()
    {
        // TinyUSB's interrupt lands on the core that initialises it, so all
        // USB, lwIP and stdio work stays here.
        tusb_init();
        stdio_usb_init();
        net::init();
        http::init(on_key);
        printf("encoder doom: open http://192.168.7.1\n");

        for (;;)
        {
            tud_task();
            net::poll();
            send_frame_if_ready();
            print_stats();
        }
    }
}  // namespace

void platform::init()
{
    queue_init(&key_queue, sizeof(KeyEvent), kKeyQueueLength);
    multicore_launch_core1_with_stack(core1_main, core1_stack, sizeof(core1_stack));
}

void platform::display_set_palette(const uint8_t* rgb) { std::memcpy(current_palette, rgb, kPaletteBytes); }

void platform::display_present(const uint8_t* pixels)
{
    if (frame_ready.load(std::memory_order_acquire))
    {
        frames_dropped.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    for (int y = 0; y < kHeight; ++y)
    {
        uint8_t* row = &slot_scanlines[y * (kWidth + 1)];
        row[0] = 0;  // PNG filter: none
        std::memcpy(row + 1, pixels + y * kWidth, kWidth);
    }
    std::memcpy(slot_palette, current_palette, kPaletteBytes);
    frame_ready.store(true, std::memory_order_release);
}

bool platform::input_poll(KeyEvent* event) { return queue_try_remove(&key_queue, event); }
