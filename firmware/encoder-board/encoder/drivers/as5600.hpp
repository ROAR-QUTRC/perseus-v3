// as5600.hpp
//
// Driver for the AMS AS5600 12-bit magnetic rotary encoder, read over I2C.

#pragma once

#include <cstddef>
#include <cstdint>

#include "hardware/i2c.h"

// Talks to a single AS5600 on an already-configured I2C bus.
//
// This class does not touch i2c_init()/gpio_set_function()/gpio_pull_up() --
// the caller owns bus setup, since the bus may end up shared with other
// devices. Construct this only after the bus is initialized.
//
// Not thread/core safe: in the target dual-core layout, this is meant to be
// owned and called exclusively from the core reading the encoder. A zero()
// request arriving over Modbus from the other core should be handed over
// via a queue/message rather than calling into this object directly from
// that core.
class As5600
{
public:
    struct Reading
    {
        uint16_t raw_counts;  // 0-4095, post zero-offset
        float degrees;
    };

    static constexpr uint8_t kDefaultAddr = 0x36;  // fixed 7-bit address, not configurable

    explicit As5600(i2c_inst_t* i2c_port, uint8_t addr = kDefaultAddr);

    // True if the sensor reports a magnet in range.
    bool magnet_detected() const;

    // Reads the current angle, adjusted by any offset set via zero().
    // Returns false (and leaves *out untouched) on an I2C error.
    bool read(Reading* out) const;

    // Takes the sensor's current raw position as the new zero point for
    // read(). This is a software offset applied on every read, not a write
    // to the sensor's OTP burn registers -- it's undone on power cycle.
    // Returns false (and leaves the offset unchanged) on an I2C error.
    bool zero();

    // Undoes zero(): read() goes back to reporting the sensor's raw angle.
    void clear_zero();

private:
    // AS5600 register map (relevant subset; see datasheet section 8)
    static constexpr uint8_t kRegStatus = 0x0B;       // magnet-detect flags
    static constexpr uint8_t kRegAngle = 0x0E;        // 12-bit filtered/scaled angle, high byte first
    static constexpr uint8_t kStatusMdBit = 1u << 5;  // MD: magnet detected
    static constexpr uint16_t kCountsRange = 4096;    // 12-bit range
    static constexpr float kCountsToDegrees = 360.0f / kCountsRange;

    bool read_register(uint8_t reg, uint8_t* dst, size_t len) const;
    bool read_raw_angle(uint16_t* counts) const;

    i2c_inst_t* i2c_port_;
    uint8_t addr_;
    uint16_t zero_offset_ = 0;
};
