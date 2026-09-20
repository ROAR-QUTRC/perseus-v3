// as5600.cpp

// TODO: Potentially add Mag too high or too low read from status register, as well as different filtering and hysterisis

#include "as5600.hpp"

As5600::As5600(i2c_inst_t* i2c_port, uint8_t addr)
    : i2c_port_(i2c_port),
      addr_(addr)
{
}

bool As5600::read_register(uint8_t reg, uint8_t* dst, size_t len) const
{
    // Write the register pointer, then read len bytes (the AS5600 auto-increments it).
    if (i2c_write_blocking(i2c_port_, addr_, &reg, 1, true) < 0)
        return false;  // NAK -- sensor not responding on the bus
    return i2c_read_blocking(i2c_port_, addr_, dst, len, false) == static_cast<int>(len);
}

bool As5600::read_raw_angle(uint16_t* counts) const
{
    // 12-bit big-endian; the top 4 bits of the high byte are unused (read 0).
    uint8_t buf[2];
    if (!read_register(kRegAngle, buf, sizeof(buf)))
        return false;
    *counts = ((static_cast<uint16_t>(buf[0]) << 8) | buf[1]) & 0x0FFFu;
    return true;
}

bool As5600::magnet_detected() const
{
    uint8_t status = 0;
    return read_register(kRegStatus, &status, 1) && (status & kStatusMdBit);
}

bool As5600::read(Reading* out) const
{
    uint16_t counts;
    if (!read_raw_angle(&counts))
        return false;
    counts = static_cast<uint16_t>((counts - zero_offset_ + kCountsRange) % kCountsRange);
    out->raw_counts = counts;
    out->degrees = counts * kCountsToDegrees;
    return true;
}

bool As5600::zero()
{
    uint16_t counts;
    if (!read_raw_angle(&counts))
        return false;
    zero_offset_ = counts;
    return true;
}

void As5600::clear_zero() { zero_offset_ = 0; }
