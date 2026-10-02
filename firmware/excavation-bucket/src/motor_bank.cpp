#include "motor_bank.hpp"

#include <Arduino.h>

#include <algorithm>
#include <cmath>

#include "bank_control_task.hpp"
#include "encoder_bus.hpp"
#include "esp_log.h"
#include "hi_can_parameter.hpp"
#include "lock.hpp"

static const char* const TAG = "motor_bank";

MotorBank::MotorBank(const bsp::pin_pair_t& driver_A_pins,
                     EncoderId driver_A_encoder_id,
                     uint8_t driver_A_encoder_group_id,
                     const bsp::pin_pair_t& driver_B_pins,
                     EncoderId driver_B_encoder_id,
                     uint8_t driver_B_encoder_group_id,
                     const gpio_num_t& current_sense_pin,
                     const gpio_num_t& fault_pin,
                     int8_t speed_direction,
                     int8_t position_direction,
                     float min_angle,
                     float max_angle, EncoderBus* encoder_bus)
    : _driver_A(driver_A_pins, driver_A_encoder_id, driver_A_encoder_group_id, encoder_bus),
      _driver_B(driver_B_pins, driver_B_encoder_id, driver_B_encoder_group_id, encoder_bus),
      _current_sense_pin(current_sense_pin),
      _fault_pin(fault_pin),
      _speed_direction(speed_direction < 0 ? -1 : 1),
      _position_direction(position_direction < 0 ? -1 : 1),
      _min_angle(std::max(min_angle, -kMaxAngle)),
      _max_angle(std::min(max_angle, kMaxAngle)),
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

// Each command wakes the control task after releasing the lock, so the task
// doesn't wake only to block on it.
void MotorBank::set_speed(const int16_t speed)
{
    {
        Lock lock(_mutex);
        _speed = speed;
        if (speed > to_duty(kSpeedDeadband) || speed < -to_duty(kSpeedDeadband))
            _mode = ControlMode::Velocity;
    }
    wake_bank_control_task();
}

void MotorBank::set_target_position(const int16_t position)
{
    // Angles don't wrap and the bank won't drive past its limits, so a target
    // outside them could never be reached.
    const float degrees = position / kPositionUnitsPerDegree;
    if (degrees < _min_angle || degrees > _max_angle)
    {
        ESP_LOGW(TAG, "SET_POSITION %.1f deg ignored: outside %.0f..%.0f", degrees, _min_angle, _max_angle);
        return;
    }
    {
        Lock lock(_mutex);
        _target_position = position;
        _mode = ControlMode::Position;
    }
    wake_bank_control_task();
}

void MotorBank::stop()
{
    {
        Lock lock(_mutex);
        _speed = 0;
        _mode = ControlMode::Velocity;
    }
    wake_bank_control_task();
}

void MotorBank::enable_homing(float bite_current, float idle_current)
{
    _home_bite_current = bite_current;
    _home_idle_current = idle_current;
}

void MotorBank::start_homing()
{
    if (!homing_enabled())
        return;
    {
        Lock lock(_mutex);
        if (_mode == ControlMode::Homing)
            return;  // already running
        _speed = 0;
        _mode = ControlMode::Homing;
    }
    wake_bank_control_task();
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
    Lock lock(_mutex);
    return _current_amps;
}

bool MotorBank::is_in_fault() { return digitalRead(_fault_pin) == LOW; }

// GET_CURRENT is in mA, what ROS's bucket_hardware decodes.
std::vector<uint8_t> MotorBank::get_current()
{
    const float milliamps = std::clamp(this->get_average_current() * 1000.0f, 0.0f,
                                       static_cast<float>(std::numeric_limits<uint16_t>::max()));
    hi_can::parameters::excavation::bucket::controller::current_t current{
        static_cast<uint16_t>(std::lround(milliamps))};
    return current.serialize_data();
};

std::vector<uint8_t> MotorBank::get_fault()
{
    hi_can::parameters::excavation::bucket::controller::status_t status{is_in_fault()};
    return status.serialize_data();
}

void MotorBank::control_tick(uint32_t now_ms)
{
    const float amps = voltage_to_current(analogReadMilliVolts(_current_sense_pin) / 1000.0f);

    ControlMode mode;
    int16_t speed;
    int16_t target;
    {
        Lock lock(_mutex);
        _current_amps = amps;
        mode = _mode;
        speed = _speed;
        target = _target_position;
    }

    int16_t output_a = 0;
    int16_t output_b = 0;

    if (mode == ControlMode::Homing)
    {
        if (!_home_active)
        {
            _home_active = true;
            _home_bites = 0;
            set_home_step(HomeStep::Open, now_ms);
            ESP_LOGI(TAG, "homing: opening until the current falls below %.2f A", _home_idle_current);
        }
        output_a = output_b = homing_output(now_ms, amps);
    }
    else
    {
        if (_home_active)
        {
            _home_active = false;
            ESP_LOGW(TAG, "homing: cancelled by a command");
        }

        // Both actuators move the same joint, so a side with no reading
        // follows the other side's encoder.
        const std::optional<float> own_a = encoder_degrees(_driver_A.encoder_id(), now_ms);
        const std::optional<float> own_b = encoder_degrees(_driver_B.encoder_id(), now_ms);
        const std::optional<float> angle_a = own_a ? own_a : own_b;
        const std::optional<float> angle_b = own_b ? own_b : own_a;

        if (mode == ControlMode::Position)
        {
            if (_last_mode != ControlMode::Position || target != _last_target)
            {
                _settled_a = false;
                _settled_b = false;
                _last_target = target;
            }

            const float target_degrees = target / kPositionUnitsPerDegree;
            output_a = position_output(target_degrees, angle_a, &_settled_a);
            output_b = position_output(target_degrees, angle_b, &_settled_b);
        }
        else
        {
            // -32768 has no positive int16 counterpart, so it flips to 32767.
            output_a = output_b =
                _speed_direction < 0 ? static_cast<int16_t>(-std::max<int32_t>(speed, -INT16_MAX)) : speed;
        }

        output_a = limit_output(output_a, angle_a);
        output_b = limit_output(output_b, angle_b);
    }
    _last_mode = mode;

    _driver_A.drive(output_a);
    _driver_B.drive(output_b);

    Lock lock(_mutex);
    _output_a = output_a;
    _output_b = output_b;
}

// Zero if `output` would drive the joint past a limit. Driving back into range
// is always allowed, and with no angle there is nothing to check against.
int16_t MotorBank::limit_output(int16_t output, std::optional<float> angle) const
{
    if (!angle || output == 0)
        return output;
    const bool raising = (output > 0) == (_position_direction > 0);
    const bool blocked = raising ? *angle >= _max_angle : *angle <= _min_angle;
    return blocked ? 0 : output;
}

// One step of the homing sequence; returns the duty for both motors.
int16_t MotorBank::homing_output(uint32_t now_ms, float amps)
{
    const int16_t open = to_duty(kHomeSpeed) * _position_direction;  // opening raises the angle
    const int16_t close = -open;
    const uint32_t elapsed = now_ms - _home_step_start_ms;

    if (is_in_fault())
        return end_homing("driver fault");

    switch (_home_step)
    {
    case HomeStep::Open:
        if (home_detect(amps <= _home_idle_current || amps >= _home_bite_current, now_ms))
        {
            // A stall here means the jaws ran into something, or are closing instead.
            if (amps >= _home_bite_current)
                return end_homing("stalled while opening");
            ESP_LOGI(TAG, "homing: open end reached (%.2f A)", amps);
            set_home_step(HomeStep::Bite, now_ms);
            return close;
        }
        if (elapsed >= kHomeTimeoutMs)
            return end_homing("current never fell to idle while opening");
        return open;

    case HomeStep::Bite:
        if (home_detect(amps >= _home_bite_current, now_ms))
        {
            ++_home_bites;
            ESP_LOGI(TAG, "homing: bite %u/%u at %.2f A", _home_bites, kHomeBites, amps);
            if (_home_bites < kHomeBites)
            {
                set_home_step(HomeStep::Backoff, now_ms);
                return open;
            }
            const bool a = encoder_bus().zero(_driver_A.encoder_id());
            const bool b = encoder_bus().zero(_driver_B.encoder_id());
            if (!a || !b)
                ESP_LOGW(TAG, "homing: zero not queued (encoder A %s, B %s)", a ? "ok" : "failed", b ? "ok" : "failed");
            set_home_step(HomeStep::Zero, now_ms);
            return close;
        }
        if (elapsed >= kHomeTimeoutMs)
            return end_homing("no current spike while closing");
        return close;

    case HomeStep::Zero:
        if (elapsed >= kHomeZeroHoldMs)
        {
            set_home_step(HomeStep::Backoff, now_ms);
            return open;
        }
        return close;

    case HomeStep::Backoff:
        if (elapsed < kHomeBackoffMs)
            return open;
        if (_home_bites >= kHomeBites)
            return end_homing();
        set_home_step(HomeStep::Bite, now_ms);
        return close;
    }
    return end_homing("bad state");
}

void MotorBank::set_home_step(HomeStep step, uint32_t now_ms)
{
    _home_step = step;
    _home_step_start_ms = now_ms;
    _home_detecting = false;
}

// True once `condition` has held for kHomeDetectMs, not counting the inrush
// at the start of the step.
bool MotorBank::home_detect(bool condition, uint32_t now_ms)
{
    if (!condition || now_ms - _home_step_start_ms < kHomeInrushMs)
    {
        _home_detecting = false;
        return false;
    }
    if (!_home_detecting)
    {
        _home_detecting = true;
        _home_detect_start_ms = now_ms;
    }
    return now_ms - _home_detect_start_ms >= kHomeDetectMs;
}

// Stops the motors and hands back to Velocity mode, unless a command already
// took over.
int16_t MotorBank::end_homing(const char* abort_reason)
{
    _home_active = false;
    {
        Lock lock(_mutex);
        if (_mode == ControlMode::Homing)
        {
            _speed = 0;
            _mode = ControlMode::Velocity;
        }
    }
    if (abort_reason)
        ESP_LOGW(TAG, "homing: aborted, %s", abort_reason);
    else
        ESP_LOGI(TAG, "homing: done, encoders zeroed at the clench");
    return 0;
}

// kPositionSpeed straight toward the target until within kHoldWindow. No
// wrap-around: target and angle are both -180..180 and the joint never crosses
// +/-180. An overshoot just drives back. Once stopped it stays stopped until
// the error passes kResumeWindow, so encoder noise can't chatter.
int16_t MotorBank::position_output(float target, std::optional<float> angle, bool* settled) const
{
    if (!angle)
    {
        *settled = false;
        return 0;  // no feedback: don't drive blind
    }

    const float error = target - *angle;
    const float magnitude = std::fabs(error);

    if (*settled && magnitude <= kResumeWindow)
        return 0;
    if (magnitude <= kHoldWindow)
    {
        *settled = true;
        return 0;
    }
    *settled = false;

    const int16_t duty = to_duty(kPositionSpeed) * _position_direction;
    return error > 0 ? duty : -duty;
}

std::optional<float> MotorBank::encoder_degrees(EncoderId id, uint32_t now_ms)
{
    EncoderReading reading;
    if (!encoder_bus().get(id, &reading) || reading.angle_age_ms(now_ms) > kFeedbackStaleMs)
        return std::nullopt;
    return reading.degrees;
}
