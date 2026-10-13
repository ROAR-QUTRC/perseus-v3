#include "modbus/protocol.hpp"

namespace modbus
{
    uint16_t crc16(const uint8_t* data, size_t len)
    {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < len; ++i)
        {
            crc ^= data[i];
            for (int bit = 0; bit < 8; ++bit)
            {
                if (crc & 0x0001)
                    crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
                else
                    crc >>= 1;
            }
        }
        return crc;
    }

    size_t append_crc(uint8_t* frame, size_t len)
    {
        const uint16_t crc = crc16(frame, len);
        frame[len] = static_cast<uint8_t>(crc & 0xFF);
        frame[len + 1] = static_cast<uint8_t>(crc >> 8);
        return len + 2;
    }

    bool crc_valid(const uint8_t* frame, size_t len)
    {
        if (len < 3)
            return false;
        const uint16_t received =
            static_cast<uint16_t>(frame[len - 2]) | (static_cast<uint16_t>(frame[len - 1]) << 8);
        return received == crc16(frame, len - 2);
    }

    uint32_t frame_gap_us(uint32_t baud_hz)
    {
        if (baud_hz == 0)
            return 1000;
        return static_cast<uint32_t>((38'500'000ull + baud_hz - 1) / baud_hz);
    }

    static size_t build_request(uint8_t* out, uint8_t slave, Function function, uint16_t a, uint16_t b)
    {
        out[0] = slave;
        out[1] = static_cast<uint8_t>(function);
        out[2] = static_cast<uint8_t>(a >> 8);
        out[3] = static_cast<uint8_t>(a & 0xFF);
        out[4] = static_cast<uint8_t>(b >> 8);
        out[5] = static_cast<uint8_t>(b & 0xFF);
        return append_crc(out, 6);
    }

    size_t build_read_request(uint8_t* out, uint8_t slave, uint16_t start, uint16_t count)
    {
        return build_request(out, slave, Function::ReadHoldingRegisters, start, count);
    }

    size_t build_write_single_request(uint8_t* out, uint8_t slave, uint16_t reg, uint16_t value)
    {
        return build_request(out, slave, Function::WriteSingleRegister, reg, value);
    }
}  // namespace modbus
