#pragma once

#include <driver/sdm.h>

#include <board_support.hpp>
#include <cstdint>

#include "hi_can_packet.hpp"
#include "motor_driver.hpp"

class MotorBank
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

    MotorBank(const bsp::pin_pair_t& driver_A_pins,
              const bsp::pin_pair_t& driver_B_pins,
              const gpio_num_t& current_sense_pin, const gpio_num_t& fault_pin);

    // delete copy/move semantics
    MotorBank(const MotorBank&) = delete;
    MotorBank(MotorBank&&) = delete;
    MotorBank& operator=(const MotorBank&) = delete;
    MotorBank& operator=(MotorBank&&) = delete;

    virtual ~MotorBank();

    void set_speed(const int16_t speed);
    void set_speed_a(const int16_t speed) { _driver_A.set_speed(speed); }
    void set_speed_b(const int16_t speed) { _driver_B.set_speed(speed); }

    std::vector<uint8_t> get_current();
    float get_average_current();
    bool is_in_fault();

private:
    MotorDriver _driver_A;
    MotorDriver _driver_B;

    const gpio_num_t _current_sense_pin;
    const gpio_num_t _fault_pin;
};
