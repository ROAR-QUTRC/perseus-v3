#pragma once

#include "motor_driver.hpp"
#include <board_support.hpp>
#include <cstdint>
#include <driver/sdm.h>

class MotorBank {
public:
  static constexpr uint16_t CURRENT_SENSE_RESISTOR = 1000;       // ohms
  static constexpr float CURRENT_SENSE_PROPORTIONALITY = 450e-6; // amps per amp

  // static constexpr float current_to_voltage(const float& current);
  // static constexpr float voltage_to_current(const float& voltage);
  static constexpr float current_to_voltage(const float &current) {
    return current * (CURRENT_SENSE_RESISTOR * CURRENT_SENSE_PROPORTIONALITY);
  }
  static constexpr float voltage_to_current(const float &voltage) {
    return voltage / (CURRENT_SENSE_RESISTOR * CURRENT_SENSE_PROPORTIONALITY);
  }

  static constexpr float MAX_VOLTAGE = 3.3f; // volts
  static constexpr float MAX_CURRENT = 6.0f; // amps

  MotorBank(const bsp::pin_pair_t &driver_A_pins,
            const bsp::pin_pair_t &driver_B_pins,
            const gpio_num_t &current_limit_pin,
            const gpio_num_t &current_sense_pin, const gpio_num_t &fault_pin);

  // delete copy/move semantics
  MotorBank(const MotorBank &) = delete;
  MotorBank(MotorBank &&) = delete;
  MotorBank &operator=(const MotorBank &) = delete;
  MotorBank &operator=(MotorBank &&) = delete;

  virtual ~MotorBank();

  void set_speed(int16_t speed_a, int16_t speed_b);
  void set_speed_a(int16_t speed) { _driver_A.set_speed(speed); }
  void set_speed_b(int16_t speed) { _driver_B.set_speed(speed); }

  float get_current_limit() const { return _current_limit; }
  void set_current_limit(float limit);
  float get_average_current();

  bool is_in_fault();

private:
  float _current_limit = 0.0f;
  sdm_channel_handle_t _current_limit_channel;

  MotorDriver _driver_A;
  MotorDriver _driver_B;

  const gpio_num_t _current_limit_pin;
  const gpio_num_t _current_sense_pin;
  const gpio_num_t _fault_pin;
};
