#pragma once

#include <cstddef>
#include <cstdint>

#include "modbus/protocol.hpp"
#include "rs485/port.hpp"

namespace modbus
{
    // Modbus RTU slave. Register access is delegated to application handlers;
    // this class only does framing, CRC, addressing and exceptions. It knows
    // nothing about which registers exist or what any of them mean.
    class Slave
    {
    public:
        // Return false to have the slave reply with an illegal-data-address
        // exception.
        using ReadFn = bool (*)(uint16_t address, uint16_t* out_value, void* ctx);
        using WriteFn = bool (*)(uint16_t address, uint16_t value, void* ctx);

        struct Counters
        {
            uint32_t frames_handled = 0;  // valid CRC and addressed to us (or broadcast)
            uint32_t crc_errors = 0;
            uint32_t short_frames = 0;
            uint32_t exceptions_sent = 0;
        };

        Slave(rs485::Port& port, uint8_t address);

        void set_read_handler(ReadFn fn, void* ctx);
        void set_write_handler(WriteFn fn, void* ctx);

        // Services at most one incoming frame. Call it regularly; it waits up to
        // one frame gap for a frame to start.
        void poll();

        uint8_t address() const { return address_; }
        const Counters& counters() const { return counters_; }

    private:
        void handle_frame(const uint8_t* frame, size_t len, bool broadcast);
        void handle_read_holding(const uint8_t* frame, size_t len, bool broadcast);
        void handle_write_single(const uint8_t* frame, size_t len, bool broadcast);
        void handle_write_multiple(const uint8_t* frame, size_t len, bool broadcast);
        void send_exception(uint8_t function, ExceptionCode code, bool broadcast);
        void send_response(uint8_t* frame, size_t len);  // appends CRC, transmits
        bool write_register(uint16_t address, uint16_t value);

        rs485::Port& port_;
        uint8_t address_;
        ReadFn read_fn_ = nullptr;
        void* read_ctx_ = nullptr;
        WriteFn write_fn_ = nullptr;
        void* write_ctx_ = nullptr;
        Counters counters_{};
    };
}  // namespace modbus
