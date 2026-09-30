#include "modbus/master.hpp"

#include <cstring>

namespace modbus
{
    Response Master::exchange(const uint8_t* request, size_t request_len, uint8_t slave, uint8_t function,
                              uint8_t* rx, size_t* rx_len)
    {
        Response r;
        const uint32_t gap_us = frame_gap_us(port_.baud_hz());
        const uint32_t start_us = clock_.now_us();

        port_.flush_rx();
        if (!port_.send(request, request_len))
        {
            r.result = Result::TransportError;
            r.timestamp_ms = clock_.now_ms();
            return r;
        }

        const size_t got = port_.receive(rx, kMaxFrameLen, response_timeout_us_, gap_us);
        r.latency_us = clock_.now_us() - start_us;
        r.timestamp_ms = clock_.now_ms();
        *rx_len = got;

        // The next request must not start until the bus has been silent for a frame gap.
        clock_.delay_us(gap_us);

        if (got == 0)
        {
            r.result = Result::Timeout;
            return r;
        }
        if (got < 5)  // shortest valid reply is an exception: addr + function + code + CRC
        {
            r.result = Result::BadFrame;
            return r;
        }
        if (!crc_valid(rx, got))
        {
            r.result = Result::CrcError;
            return r;
        }
        if (rx[0] != slave)
        {
            r.result = Result::BadFrame;
            return r;
        }
        if (rx[1] == (function | 0x80))
        {
            if (got != 5)
            {
                r.result = Result::BadFrame;
                return r;
            }
            r.result = Result::ExceptionReply;
            r.exception_code = rx[2];
            return r;
        }
        if (rx[1] != function)
        {
            r.result = Result::BadFrame;
            return r;
        }

        r.result = Result::Ok;
        return r;
    }

    Response Master::read_holding(uint8_t slave, uint16_t start, uint16_t count)
    {
        if (slave == kBroadcastAddress || count == 0 || count > kMaxResponseRegs)
        {
            Response r;
            r.result = Result::InvalidRequest;
            return r;
        }

        constexpr uint8_t kFn = static_cast<uint8_t>(Function::ReadHoldingRegisters);
        uint8_t request[8];
        const size_t request_len = build_read_request(request, slave, start, count);

        uint8_t rx[kMaxFrameLen];
        size_t rx_len = 0;
        Response r = exchange(request, request_len, slave, kFn, rx, &rx_len);
        if (r.result != Result::Ok)
            return r;

        // addr + function + byte-count + data + 2 CRC
        if (rx[2] != count * 2 || rx_len != static_cast<size_t>(3 + count * 2 + 2))
        {
            r.result = Result::BadFrame;
            return r;
        }

        r.reg_count = static_cast<uint8_t>(count);
        for (uint16_t i = 0; i < count; ++i)
            r.regs[i] = static_cast<uint16_t>((rx[3 + i * 2] << 8) | rx[4 + i * 2]);
        return r;
    }

    Response Master::write_single(uint8_t slave, uint16_t reg, uint16_t value)
    {
        uint8_t request[8];
        const size_t request_len = build_write_single_request(request, slave, reg, value);

        if (slave == kBroadcastAddress)
        {
            Response r;
            const uint32_t start_us = clock_.now_us();
            port_.flush_rx();
            if (!port_.send(request, request_len))
            {
                r.result = Result::TransportError;
                r.timestamp_ms = clock_.now_ms();
                return r;
            }
            // No reply is coming, so nothing else paces the next request: wait out
            // the inter-frame gap here so callers can't forget to.
            clock_.delay_us(frame_gap_us(port_.baud_hz()));
            r.result = Result::Ok;
            r.latency_us = clock_.now_us() - start_us;
            r.timestamp_ms = clock_.now_ms();
            return r;
        }

        constexpr uint8_t kFn = static_cast<uint8_t>(Function::WriteSingleRegister);
        uint8_t rx[kMaxFrameLen];
        size_t rx_len = 0;
        Response r = exchange(request, request_len, slave, kFn, rx, &rx_len);
        if (r.result != Result::Ok)
            return r;

        // A successful 0x06 reply is an exact echo of the request.
        if (rx_len != request_len || std::memcmp(rx, request, request_len) != 0)
            r.result = Result::BadFrame;
        return r;
    }
}  // namespace modbus
