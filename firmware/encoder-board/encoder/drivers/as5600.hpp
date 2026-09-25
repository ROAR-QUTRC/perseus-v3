// as5600.hpp
//
// Driver for the AMS AS5600 12-bit magnetic rotary encoder (I2C).

#pragma once

#include <cstddef>
#include <cstdint>

#include "hardware/i2c.h"

// The caller owns I2C bus setup and must construct this after the bus is
// initialized. Not thread/core safe: call it only from the task that owns the
// sensor (zero requests from Modbus go through a queue, not a direct call).
class As5600
{
public:
    struct Reading
    {
        uint16_t raw_counts;  // 0-4095, post zero-offset
        float degrees;
    };

    static constexpr uint8_t kDefaultAddr = 0x36;  // fixed 7-bit address

    explicit As5600(i2c_inst_t* i2c_port, uint8_t addr = kDefaultAddr);

    bool magnet_detected() const;

    // Returns false (leaving *out untouched) on an I2C error.
    bool read(Reading* out) const;

    // Makes the current position the new zero. A software offset applied on
    // every read, not an OTP burn, so it is lost on power cycle. Returns false
    // (offset unchanged) on an I2C error.
    // TODO: an encoder reboot silently moves the reference back to raw 0; persist
    // the offset (flash) or have the master re-zero after an encoder reset.
    bool zero();

    void clear_zero();

private:
    static constexpr uint8_t kRegStatus = 0x0B;
    static constexpr uint8_t kRegAngle = 0x0E;        // 12-bit angle, high byte first
    static constexpr uint8_t kStatusMdBit = 1u << 5;  // MD: magnet detected
    static constexpr uint16_t kCountsRange = 4096;
    static constexpr float kCountsToDegrees = 360.0f / kCountsRange;

    bool read_register(uint8_t reg, uint8_t* dst, size_t len) const;
    bool read_raw_angle(uint16_t* counts) const;

    i2c_inst_t* i2c_port_;
    uint8_t addr_;
    uint16_t zero_offset_ = 0;
};
