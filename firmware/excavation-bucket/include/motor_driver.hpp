#pragma once

#include <board_support.hpp>

#include "shared_memory.hpp"

/**
 * @brief A class responsible for comparing current position to target position and driving to close the gap
 * @details Call this class for individual joint based readings and configurations. Each motor driver owns a shared memory that is set by the packet_manager and read by the driver
 */
class MotorDriver : public ExcavationJoint
{
public:
    enum class direction
    {
        FORWARD,
        STOPPED,
        BACKWARD
    };

    MotorDriver(const bsp::pin_pair_t& pins, EncoderId encoder_id, uint8_t encoder_group_id, EncoderBus* encoder_bus);
    virtual ~MotorDriver();

    void set_speed(int16_t speed) override;
    void set_target_position(int16_t position) override;
    int16_t get_current_position() const override;
    EncoderId encoder_id() const { return _motor_memory.encoder_id(); }

    void monitor_and_move(void) override;

    // Writes the PWM. Only the bank control task calls this; +/-32767 = full.
    void drive(int16_t speed);

private:
    direction _prev_direction = direction::STOPPED;
    bsp::pin_pair_t _pins;
    MotorMemory _motor_memory;
};
