// http_server.hpp
//
//   GET  /        the page (index.html, compiled into flash)
//   GET  /stream  multipart/x-mixed-replace of PNG frames, shown by the page's <img>
//   POST /key     "1<code>" key down, "0<code>" key up, "R" release all (<code> = KeyboardEvent.code)
//
// All on core 1, like the rest of lwIP.

#pragma once

#include <cstddef>
#include <cstdint>

namespace http
{
    using KeyHandler = void (*)(const char* body, size_t length);

    void init(KeyHandler on_key);

    // A viewer is connected and the last frame is fully queued in lwIP.
    bool stream_ready();

    // Queues one frame. lwIP copies it as it goes, so `png` may be reused once
    // stream_ready() is true again; encoding the next frame then overlaps
    // with the tail of this one on the wire.
    void stream_send(const uint8_t* png, size_t size);

    // Frames sent since the last call, for the stats line.
    uint32_t take_frames_sent();
}  // namespace http
