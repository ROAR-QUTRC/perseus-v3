#include "motor_bank.hpp"

#include <Arduino.h>

#include <algorithm>
#include <cmath>

#include "encoder_bus.hpp"
#include "hi_can_parameter.hpp"
#include "lock.hpp"

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
      _fault_pin(fault_pin),
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

void MotorBank::monitor_and_move(void)
{
    _driver_A.monitor_and_move();
    _driver_B.monitor_and_move();
}

void MotorBank::set_speed(const int16_t speed)
{
    Lock lock(_mutex);
    _speed = speed;
    if (speed > to_duty(kSpeedDeadband) || speed < -to_duty(kSpeedDeadband))
        _mode = ControlMode::Velocity;
}

void MotorBank::set_target_position(const int16_t position)
{
    Lock lock(_mutex);
    _target_position = position;
    _mode = ControlMode::Position;
}

void MotorBank::stop()
{
    Lock lock(_mutex);
    _speed = 0;
    _mode = ControlMode::Velocity;
}

// Falls back to the last cached angles if neither encoder is fresh.
// TODO: that fallback hides a dead encoder from ROS; stop sending GET_POSITION instead.
int16_t MotorBank::get_current_position() const
{
    const uint32_t now = encoder_bus().now_ms();
    const std::optional<float> a = encoder_degrees(_driver_A.encoder_id(), now);
    const std::optional<float> b = encoder_degrees(_driver_B.encoder_id(), now);
    if (a || b)
    {
        const float degrees = (a && b) ? (*a + *b) / 2.0f : (a ? *a : *b);
        return static_cast<int16_t>(std::lround(degrees * kPositionUnitsPerDegree));
    }
    return static_cast<int16_t>((_driver_A.get_current_position() + _driver_B.get_current_position()) / 2);
}

MotorBank::Status MotorBank::get_status() const
{
    Lock lock(_mutex);
    return {_mode, _speed, _target_position, _output_a, _output_b};
}

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

std::vector<uint8_t> MotorBank::get_fault()
{
    hi_can::parameters::excavation::bucket::controller::status_t status{is_in_fault()};
    return status.serialize_data();
}

void MotorBank::control_tick(uint32_t now_ms)
{
    ControlMode mode;
    int16_t speed;
    int16_t target;
    {
        Lock lock(_mutex);
        mode = _mode;
        speed = _speed;
        target = _target_position;
    }

    int16_t output_a = speed;
    int16_t output_b = speed;

    if (mode == ControlMode::Position)
    {
        if (_last_mode != ControlMode::Position || target != _last_target)
        {
            _settled_a = false;
            _settled_b = false;
            _last_target = target;
        }

        // Both actuators move the same joint, so a side with no reading
        // follows the other side's encoder.
        const float target_degrees = target / kPositionUnitsPerDegree;
        const std::optional<float> angle_a = encoder_degrees(_driver_A.encoder_id(), now_ms);
        const std::optional<float> angle_b = encoder_degrees(_driver_B.encoder_id(), now_ms);
        output_a = position_output(target_degrees, angle_a ? angle_a : angle_b, &_settled_a);
        output_b = position_output(target_degrees, angle_b ? angle_b : angle_a, &_settled_b);
    }
    _last_mode = mode;

    _driver_A.drive(output_a);
    _driver_B.drive(output_b);

    Lock lock(_mutex);
    _output_a = output_a;
    _output_b = output_b;
}

// kPositionSpeed toward the target, the shortest way round, until within
// kHoldWindow. An overshoot just drives back. Once stopped it stays stopped
// until the error passes kResumeWindow, so encoder noise can't chatter.
int16_t MotorBank::position_output(float target, std::optional<float> angle, bool* settled)
{
    if (!angle)
    {
        *settled = false;
        return 0;  // no feedback: don't drive blind
    }

    const float error = angle_difference(target, *angle);
    const float magnitude = std::fabs(error);

    if (*settled && magnitude <= kResumeWindow)
        return 0;
    if (magnitude <= kHoldWindow)
    {
        *settled = true;
        return 0;
    }
    *settled = false;

    const int16_t duty = to_duty(kPositionSpeed) * kDriveDirection;
    return error > 0 ? duty : -duty;
}

std::optional<float> MotorBank::encoder_degrees(EncoderId id, uint32_t now_ms)
{
    EncoderReading reading;
    if (!encoder_bus().get(id, &reading) || reading.angle_age_ms(now_ms) > kFeedbackStaleMs)
        return std::nullopt;
    return reading.degrees;
}

float MotorBank::angle_difference(float to, float from)
{
    float difference = std::fmod(to - from + 180.0f, 360.0f);
    if (difference <= 0.0f)
        difference += 360.0f;
    return difference - 180.0f;
}
