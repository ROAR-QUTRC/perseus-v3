#include "motor_bank.hpp"
#include <Arduino.h>
#include <format>

MotorBank::MotorBank(const bsp::pin_pair_t &driver_A_pins,
                     const bsp::pin_pair_t &driver_B_pins,
                     const gpio_num_t &current_limit_pin,
                     const gpio_num_t &current_sense_pin,
                     const gpio_num_t &fault_pin)
    : _driver_A(driver_A_pins), _driver_B(driver_B_pins),
      _current_limit_pin(current_limit_pin),
      _current_sense_pin(current_sense_pin), _fault_pin(fault_pin) {
  pinMode(_current_limit_pin, OUTPUT);
  pinMode(_current_sense_pin, INPUT);
  pinMode(_fault_pin, INPUT_PULLUP);

  set_speed(0, 0);

  sdm_config_t config = {
      .gpio_num = _current_limit_pin,
      .clk_src = SDM_CLK_SRC_DEFAULT,
      .sample_rate_hz = 1000 * 1000,
  };
  esp_err_t err = sdm_new_channel(&config, &_current_limit_channel);
  if (err != ESP_OK) {
    throw std::runtime_error(
        std::format("Failed to create SDM channel: {}", esp_err_to_name(err)));
  }

  sdm_channel_enable(_current_limit_channel);
  if (err != ESP_OK) {
    throw std::runtime_error(
        std::format("Failed to enable SDM channel: {}", esp_err_to_name(err)));
  }
  set_current_limit(MAX_CURRENT);
}

MotorBank::~MotorBank() {
  sdm_channel_set_pulse_density(_current_limit_channel, -128);
  sdm_channel_disable(_current_limit_channel);
  sdm_del_channel(_current_limit_channel);
  pinMode(_current_limit_pin, INPUT);
  pinMode(_current_sense_pin, INPUT);
  pinMode(_fault_pin, INPUT_PULLUP);
}

void MotorBank::set_speed(int16_t speed_a, int16_t speed_b) {
  _driver_A.set_speed(speed_a);
  _driver_B.set_speed(speed_b);
}

void MotorBank::set_current_limit(float limit) {
  if (limit > MAX_CURRENT)
    limit = MAX_CURRENT;
  _current_limit = limit;
  float voltage = current_to_voltage(limit);
  const int8_t pwm_value = static_cast<int8_t>(
      std::clamp((voltage * 255 / MAX_VOLTAGE) - 128,
                 static_cast<float>(std::numeric_limits<int8_t>::min()),
                 static_cast<float>(std::numeric_limits<int8_t>::max())));
  sdm_channel_set_pulse_density(_current_limit_channel, pwm_value);
  printf(std::format("Set current limit to {:.02f} {:.02f} {}\n", limit,
                     voltage, pwm_value)
             .c_str());
}

float MotorBank::get_average_current() {
  return voltage_to_current(analogReadMilliVolts(_current_sense_pin) / 1000.0f);
}

bool MotorBank::is_in_fault() { return digitalRead(_fault_pin) == LOW; }
