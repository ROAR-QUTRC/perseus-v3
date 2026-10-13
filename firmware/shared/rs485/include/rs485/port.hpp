#pragma once

#include <cstddef>
#include <cstdint>

namespace rs485
{
    // Half-duplex RS485 physical layer: a byte pipe plus the line's baud rate.
    // One implementation per chip. Protocols such as modbus-core are written
    // against this interface and never see the hardware.
    class Port
    {
    public:
        virtual ~Port() = default;

        // Transmit one whole frame. Must not return until the last stop bit is
        // on the wire and the driver is back in receive mode.
        virtual bool send(const uint8_t* data, size_t len) = 0;

        // Wait up to first_byte_timeout_us for the first byte, then keep reading
        // until frame_gap_us of silence or `cap` bytes. Returns the byte count,
        // 0 if nothing arrived.
        virtual size_t receive(uint8_t* buf, size_t cap, uint32_t first_byte_timeout_us,
                               uint32_t frame_gap_us) = 0;

        // Discard anything already received.
        virtual void flush_rx() = 0;

        virtual uint32_t baud_hz() const = 0;
    };
}  // namespace rs485
