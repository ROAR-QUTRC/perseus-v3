#include "motor_bank.hpp"

#include <Arduino.h>

#include "encoder_bus.hpp"
#include "hi_can_parameter.hpp"

MotorBank::MotorBank(const bsp::pin_pair_t& driver_A_pins,
                     EncoderId driver_A_encoder_id,
                     uint8_t driver_A_encoder_group_id,
                     const bsp::pin_pair_t& driver_B_pins,
                     EncoderId driver_B_encoder_id,
                     uint8_t driver_B_encoder_group_id,
                     const gpio_num_t& current_sense_pin,
                     const gpio_num_t& fault_pin, EncoderBus* encoder_bus)
    : _driver_A(driver_A_pins, driver_A_encoder_id, driver_A_encoder_group_id, encoder_bus),
      _driver_B(driver_B_pins, driver_B_encoder_id, driver_B_encoder_group_id, encoder_bus),
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

void MotorBank::monitor_and_move(void)
{
    _driver_A.monitor_and_move();
    _driver_B.monitor_and_move();
}

void MotorBank::set_speed(const int16_t speed)
{
    _driver_A.set_speed(speed);
    _driver_B.set_speed(speed);
}

void MotorBank::set_target_position(const int16_t position)
{
    _driver_A.set_target_position(position);
    _driver_B.set_target_position(position);
}

int16_t MotorBank::get_current_position() const
{
    int16_t A_position = _driver_A.get_current_position();
    int16_t B_position = _driver_B.get_current_position();

    return static_cast<int16_t>((A_position + B_position) / 2);
}

void MotorBank::set_speed_a(const int16_t speed) { _driver_A.set_speed(speed); }
void MotorBank::set_speed_b(const int16_t speed) { _driver_B.set_speed(speed); }
void MotorBank::set_target_position_a(const int16_t position) { _driver_A.set_target_position(position); }
void MotorBank::set_target_position_b(const int16_t position) { _driver_B.set_target_position(position); }
int16_t MotorBank::get_current_position_a() const { return _driver_A.get_current_position(); }
int16_t MotorBank::get_current_position_b() const { return _driver_B.get_current_position(); }
MotorDriver& MotorBank::get_driver_A() { return _driver_A; }
MotorDriver& MotorBank::get_driver_B() { return _driver_B; }

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
