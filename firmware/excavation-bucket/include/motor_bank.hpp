#pragma once

#include <driver/sdm.h>
#include "shared_memory.hpp"

#include <board_support.hpp>
#include <cstdint>

#include "hi_can_packet.hpp"
#include "motor_driver.hpp"

/**
 * @brief A class responsible for managing pair of motors (Left = A, Right = B) (TODO: check this and update)
 * @details This class provides an interface to control both motors as a pair and the individual motors separately.
 */
class MotorBank : public ExcavationJoint
{
public:
    static constexpr uint16_t CURRENT_SENSE_RESISTOR = 1000;        // ohms
    static constexpr float CURRENT_SENSE_PROPORTIONALITY = 450e-6;  // amps per amp

    static constexpr float voltage_to_current(const float& voltage)
    {
        return voltage / (CURRENT_SENSE_RESISTOR * CURRENT_SENSE_PROPORTIONALITY);
    }

    static constexpr float MAX_VOLTAGE = 3.3f;  // volts
    static constexpr float MAX_CURRENT = 6.0f;  // amps

    MotorBank(const bsp::pin_pair_t& driver_A_pins, EncoderId driver_A_encoder_id, uint8_t driver_A_encoder_group_id,
              const bsp::pin_pair_t& driver_B_pins, EncoderId driver_B_encoder_id, uint8_t driver_B_encoder_group_id,
              const gpio_num_t& current_sense_pin, const gpio_num_t& fault_pin, EncoderBus* encoder_bus);

    // delete copy/move semantics
    MotorBank(const MotorBank&) = delete;
    MotorBank(MotorBank&&) = delete;
    MotorBank& operator=(const MotorBank&) = delete;
    MotorBank& operator=(MotorBank&&) = delete;

    virtual ~MotorBank();

    // the function to be continually called to update motor status and control signals
    void monitor_and_move(void) override;
    
    // whole bank setting (applies to both motors)
    void set_speed(const int16_t speed) override;
    void set_target_position(const int16_t position) override;
    int16_t get_current_position() const override;

    // individual motors
    void set_speed_a(const int16_t speed);
    void set_speed_b(const int16_t speed);
    void set_target_position_a(const int16_t position);
    void set_target_position_b(const int16_t position);
    int16_t get_current_position_a() const;
    int16_t get_current_position_b() const;
    MotorDriver& get_driver_A(void);
    MotorDriver& get_driver_B(void);

    std::vector<uint8_t> get_current();
    float get_average_current();
    bool is_in_fault();

private:
    MotorDriver _driver_A;
    MotorDriver _driver_B;

    const gpio_num_t _current_sense_pin;
    const gpio_num_t _fault_pin;
};
