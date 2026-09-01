#pragma once

#include <board_support.hpp>

class MotorDriver
{
public:
    enum class direction
    {
        FORWARD,
        STOPPED,
        BACKWARD
    };

    MotorDriver(const bsp::pin_pair_t& pins);
    virtual ~MotorDriver();

    void set_speed(int16_t speed);

private:
    direction _prev_direction = direction::STOPPED;

    bsp::pin_pair_t _pins;
};
