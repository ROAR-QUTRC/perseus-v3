#include "motor_bank.hpp"

#include <Arduino.h>

#include "hi_can_parameter.hpp"

MotorBank::MotorBank(const bsp::pin_pair_t& driver_A_pins,
                     const bsp::pin_pair_t& driver_B_pins,
                     const gpio_num_t& current_sense_pin,
                     const gpio_num_t& fault_pin,
                     const EncoderId encoder_left,
                     const EncoderId encoder_right)
    : _driver_A(driver_A_pins),
      _driver_B(driver_B_pins),
      _current_sense_pin(current_sense_pin),
      _fault_pin(fault_pin),
      _encoder_left(encoder_left),
      _encoder_right(encoder_right),
      _position_pid(kControlPeriodMs / 1000.0),
      _mutex(xSemaphoreCreateMutex())
{
    pinMode(_current_sense_pin, INPUT);
    pinMode(_fault_pin, INPUT_PULLUP);

    set_speed(0);
}

MotorBank::~MotorBank()
{
    pinMode(_current_sense_pin, INPUT);
    pinMode(_fault_pin, INPUT_PULLUP);
    vSemaphoreDelete(_mutex);
}

void MotorBank::set_speed(const int16_t speed)
{
    Lock lock(_mutex);
    _mode = ControlMode::OPEN_LOOP_SPEED;
    _driver_A.set_speed(speed);
    _driver_B.set_speed(speed);
}

void MotorBank::set_position_setpoint(const int16_t setpoint)
{
    Lock lock(_mutex);
    if (_mode != ControlMode::CLOSED_LOOP_POSITION)
    {
        _position_pid.reset();
        _mode = ControlMode::CLOSED_LOOP_POSITION;
    }
    _position_setpoint = setpoint;
}

void MotorBank::set_pid_gains(const PidController::Gains& gains)
{
    _position_pid.set_gains(gains);
}

void MotorBank::control_tick(const uint32_t now_ms)
{
    Lock lock(_mutex);
    if (_mode != ControlMode::CLOSED_LOOP_POSITION)
        return;

    int16_t measured;
    if (!average_raw_counts(now_ms, &measured))
    {
        // Feedback fault: neither encoder has fresh, valid data. Fail safe
        // exactly like the SET_SPEED/SET_POSITION CAN watchdogs do on a stale
        // command.
        _position_pid.reset();
        _mode = ControlMode::OPEN_LOOP_SPEED;
        _driver_A.set_speed(0);
        _driver_B.set_speed(0);
        return;
    }

    const int16_t output = _position_pid.compute(_position_setpoint, measured);
    _driver_A.set_speed(output);
    _driver_B.set_speed(output);
}

bool MotorBank::average_raw_counts(const uint32_t now_ms, int16_t* out) const
{
    EncoderReading left{};
    EncoderReading right{};
    const bool have_left = encoder_bus().get(_encoder_left, &left) && left.angle_age_ms(now_ms) < kFeedbackStaleMs;
    const bool have_right = encoder_bus().get(_encoder_right, &right) && right.angle_age_ms(now_ms) < kFeedbackStaleMs;

    if (!have_left && !have_right)
        return false;

    uint32_t sum = 0;
    uint32_t count = 0;
    if (have_left)
    {
        sum += left.raw_counts;
        ++count;
    }
    if (have_right)
    {
        sum += right.raw_counts;
        ++count;
    }

    *out = static_cast<int16_t>(sum / count);
    return true;
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

std::vector<uint8_t> MotorBank::get_fault()
{
    hi_can::parameters::excavation::bucket::controller::status_t status{is_in_fault()};
    return status.serialize_data();
}

std::vector<uint8_t> MotorBank::get_position()
{
    int16_t counts = 0;
    average_raw_counts(encoder_bus().now_ms(), &counts);
    hi_can::parameters::excavation::bucket::controller::position_t position{counts};
    return position.serialize_data();
}
