#include "motor_bank.hpp"

#include <Arduino.h>

#include "hi_can_parameter.hpp"

MotorBank::MotorBank(const bsp::pin_pair_t& driver_A_pins,
                     const bsp::pin_pair_t& driver_B_pins,
                     const gpio_num_t& current_sense_pin,
                     const gpio_num_t& fault_pin)
    : _driver_A(driver_A_pins),
      _driver_B(driver_B_pins),
      _current_sense_pin(current_sense_pin),
      _fault_pin(fault_pin)
{
    pinMode(_current_sense_pin, INPUT);
    pinMode(_fault_pin, INPUT_PULLUP);

    set_speed(0);
}

MotorBank::~MotorBank()
{
    pinMode(_current_sense_pin, INPUT);
    pinMode(_fault_pin, INPUT_PULLUP);
}

void MotorBank::set_speed(const int16_t speed)
{
    _driver_A.set_speed(speed);
    _driver_B.set_speed(speed);
}

float MotorBank::get_average_current()
{
    return voltage_to_current(analogReadMilliVolts(_current_sense_pin) / 1000.0f);
}

bool MotorBank::is_in_fault() { return digitalRead(_fault_pin) == LOW; }

std::vector<uint8_t> MotorBank::get_current()
{
    float average_current = this->get_average_current();
    float clamped = std::clamp(average_current,
                               static_cast<float>(std::numeric_limits<uint16_t>::min()),
                               static_cast<float>(std::numeric_limits<uint16_t>::max()));
    hi_can::parameters::excavation::bucket::controller::current_t current{static_cast<uint16_t>(clamped)};
    return current.serialize_data();
};
