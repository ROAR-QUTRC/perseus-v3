#pragma once

#include <cstddef>
#include <cstdint>

namespace modbus
{
    inline constexpr size_t kMaxFrameLen = 64;  // far more than any register map here needs
    inline constexpr uint8_t kBroadcastAddress = 0;

    enum class Function : uint8_t
    {
        ReadHoldingRegisters = 0x03,
        WriteSingleRegister = 0x06,
        WriteMultipleRegisters = 0x10,
    };

    enum class ExceptionCode : uint8_t
    {
        IllegalFunction = 0x01,
        IllegalDataAddress = 0x02,
        IllegalDataValue = 0x03,
        SlaveDeviceFailure = 0x04,
    };

    // Modbus CRC16, polynomial 0xA001.
    uint16_t crc16(const uint8_t* data, size_t len);

    // Writes the CRC (low byte first) at frame[len]; returns len + 2.
    size_t append_crc(uint8_t* frame, size_t len);

    // `len` includes the two CRC bytes.
    bool crc_valid(const uint8_t* frame, size_t len);

    // 3.5 character times at 11 bits per character. Keeps shrinking with baud
    // rather than flooring at the spec's 1.75ms, since both ends are ours.
    uint32_t frame_gap_us(uint32_t baud_hz);

    // `out` must hold at least 8 bytes; both return the frame length with CRC.
    size_t build_read_request(uint8_t* out, uint8_t slave, uint16_t start, uint16_t count);
    size_t build_write_single_request(uint8_t* out, uint8_t slave, uint16_t reg, uint16_t value);
}  // namespace modbus
