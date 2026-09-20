#include "modbus/slave.hpp"

namespace modbus
{
    Slave::Slave(rs485::Port& port, uint8_t address)
        : port_(port),
          address_(address)
    {
    }

    void Slave::set_read_handler(ReadFn fn, void* ctx)
    {
        read_fn_ = fn;
        read_ctx_ = ctx;
    }

    void Slave::set_write_handler(WriteFn fn, void* ctx)
    {
        write_fn_ = fn;
        write_ctx_ = ctx;
    }

    void Slave::poll()
    {
        const uint32_t gap_us = frame_gap_us(port_.baud_hz());

        uint8_t frame[kMaxFrameLen];
        const size_t len = port_.receive(frame, sizeof(frame), gap_us, gap_us);

        if (len == 0)
            return;
        if (len < 4)  // shortest valid frame: addr + function + 2-byte CRC
        {
            ++counters_.short_frames;
            return;
        }
        if (!crc_valid(frame, len))
        {
            ++counters_.crc_errors;  // per spec, discard silently, no reply
            return;
        }

        const uint8_t addr = frame[0];
        const bool broadcast = (addr == kBroadcastAddress);
        if (addr != address_ && !broadcast)
            return;

        ++counters_.frames_handled;
        handle_frame(frame, len - 2, broadcast);
    }

    void Slave::handle_frame(const uint8_t* frame, size_t len, bool broadcast)
    {
        const uint8_t function = frame[1];
        switch (function)
        {
        case static_cast<uint8_t>(Function::ReadHoldingRegisters):
            handle_read_holding(frame, len, broadcast);
            break;
        case static_cast<uint8_t>(Function::WriteSingleRegister):
            handle_write_single(frame, len, broadcast);
            break;
        case static_cast<uint8_t>(Function::WriteMultipleRegisters):
            handle_write_multiple(frame, len, broadcast);
            break;
        default:
            send_exception(function, ExceptionCode::IllegalFunction, broadcast);
            break;
        }
    }

    void Slave::handle_read_holding(const uint8_t* frame, size_t len, bool broadcast)
    {
        constexpr uint8_t kFn = static_cast<uint8_t>(Function::ReadHoldingRegisters);
        if (len != 6)
        {
            send_exception(kFn, ExceptionCode::IllegalDataAddress, broadcast);
            return;
        }

        const uint16_t start = (static_cast<uint16_t>(frame[2]) << 8) | frame[3];
        const uint16_t count = (static_cast<uint16_t>(frame[4]) << 8) | frame[5];

        // 5 = addr + function + byte-count + 2 CRC bytes.
        if (count == 0 || count > (kMaxFrameLen - 5) / 2)
        {
            send_exception(kFn, ExceptionCode::IllegalDataAddress, broadcast);
            return;
        }

        uint8_t response[kMaxFrameLen];
        response[0] = address_;
        response[1] = kFn;
        response[2] = static_cast<uint8_t>(count * 2);

        for (uint16_t i = 0; i < count; ++i)
        {
            uint16_t value = 0;
            if (!read_fn_ || !read_fn_(static_cast<uint16_t>(start + i), &value, read_ctx_))
            {
                send_exception(kFn, ExceptionCode::IllegalDataAddress, broadcast);
                return;
            }
            response[3 + i * 2] = static_cast<uint8_t>(value >> 8);
            response[4 + i * 2] = static_cast<uint8_t>(value & 0xFF);
        }

        if (broadcast)
            return;  // spec: never reply to a broadcast

        send_response(response, 3u + count * 2u);
    }

    void Slave::handle_write_single(const uint8_t* frame, size_t len, bool broadcast)
    {
        constexpr uint8_t kFn = static_cast<uint8_t>(Function::WriteSingleRegister);
        if (len != 6)
        {
            send_exception(kFn, ExceptionCode::IllegalDataAddress, broadcast);
            return;
        }

        const uint16_t address = (static_cast<uint16_t>(frame[2]) << 8) | frame[3];
        const uint16_t value = (static_cast<uint16_t>(frame[4]) << 8) | frame[5];

        if (!write_register(address, value))
        {
            send_exception(kFn, ExceptionCode::IllegalDataAddress, broadcast);
            return;
        }

        if (broadcast)
            return;

        // Standard 0x06 response: an echo of the request.
        uint8_t response[8];
        for (size_t i = 0; i < 6; ++i)
            response[i] = frame[i];
        send_response(response, 6);
    }

    void Slave::handle_write_multiple(const uint8_t* frame, size_t len, bool broadcast)
    {
        constexpr uint8_t kFn = static_cast<uint8_t>(Function::WriteMultipleRegisters);
        if (len < 7)
        {
            send_exception(kFn, ExceptionCode::IllegalDataAddress, broadcast);
            return;
        }

        const uint16_t start = (static_cast<uint16_t>(frame[2]) << 8) | frame[3];
        const uint16_t count = (static_cast<uint16_t>(frame[4]) << 8) | frame[5];
        const uint8_t byte_count = frame[6];

        if (count == 0 || byte_count != count * 2 || len != static_cast<size_t>(7 + byte_count))
        {
            send_exception(kFn, ExceptionCode::IllegalDataAddress, broadcast);
            return;
        }

        for (uint16_t i = 0; i < count; ++i)
        {
            const uint16_t value = (static_cast<uint16_t>(frame[7 + i * 2]) << 8) | frame[8 + i * 2];
            if (!write_register(static_cast<uint16_t>(start + i), value))
            {
                send_exception(kFn, ExceptionCode::IllegalDataAddress, broadcast);
                return;
            }
        }

        if (broadcast)
            return;

        uint8_t response[8];
        response[0] = address_;
        response[1] = kFn;
        response[2] = frame[2];
        response[3] = frame[3];
        response[4] = frame[4];
        response[5] = frame[5];
        send_response(response, 6);
    }

    bool Slave::write_register(uint16_t address, uint16_t value)
    {
        if (!write_fn_)
            return false;
        return write_fn_(address, value, write_ctx_);
    }

    void Slave::send_exception(uint8_t function, ExceptionCode code, bool broadcast)
    {
        if (broadcast)
            return;  // spec: never reply to a broadcast, not even with an exception

        uint8_t response[5];
        response[0] = address_;
        response[1] = function | 0x80;
        response[2] = static_cast<uint8_t>(code);
        ++counters_.exceptions_sent;
        send_response(response, 3);
    }

    void Slave::send_response(uint8_t* frame, size_t len)
    {
        const size_t total = append_crc(frame, len);
        port_.send(frame, total);
    }
}  // namespace modbus
