#pragma once

#include <board_support.hpp>
#include <cstdint>

/**
 * @brief One motor driver: PWM on one pin of the pair sets the speed while the other is held low.
 */
class MotorDriver
{
public:
    explicit MotorDriver(const bsp::pin_pair_t& pins);
    ~MotorDriver();

    // Writes the PWM. Only the bank control task calls this; +/-32767 = full.
    void drive(int16_t speed);

private:
    enum class direction
    {
        FORWARD,
        STOPPED,
        BACKWARD
    };

    direction _prev_direction = direction::STOPPED;
    bsp::pin_pair_t _pins;
};
