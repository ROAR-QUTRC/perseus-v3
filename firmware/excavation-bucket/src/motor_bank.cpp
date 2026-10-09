#include "motor_bank.hpp"

#include <Arduino.h>

#include <algorithm>
#include <cmath>

#include "esp_log.h"
#include "freertos/task.h"
#include "lock.hpp"

namespace
{
    const char* const TAG = "motor_bank";

    // Below EncoderBus's RS485 tasks (priority 5, latency-sensitive UART I/O
    // position control depends on), above the Arduino loop task (priority 1,
    // CAN handling).
    constexpr UBaseType_t kTaskPriority = 4;
    constexpr BaseType_t kTaskCore = 0;  // same core as EncoderBus
    constexpr uint32_t kTaskStackBytes = 4096;

    std::array<MotorBank*, kBankCount> g_banks{};
    TaskHandle_t g_task = nullptr;

    // Runs the next control tick now, so a new command reaches the motors
    // without waiting out the period. No-op before the task has started.
    void wake_control_task()
    {
        if (g_task)
            xTaskNotifyGive(g_task);
    }

    constexpr bool config_valid(const BankConfig& bank)
    {
        const auto is_direction = [](int8_t direction)
        { return direction == 1 || direction == -1; };
        return is_direction(bank.speed_direction) && is_direction(bank.position_direction) &&
               bank.min_angle >= -MotorBank::kMaxAngle && bank.max_angle <= MotorBank::kMaxAngle &&
               bank.min_angle < bank.max_angle &&
               (bank.home_bite_current == 0.0f || bank.home_idle_current < bank.home_bite_current);
    }
    static_assert(std::ranges::all_of(kBanks, [](const BankConfig* bank)
                                      { return config_valid(*bank); }),
                  "excavation_config.hpp: each direction must be 1 or -1, min_angle < max_angle within -180..180, "
                  "and home_idle_current below home_bite_current");
}  // namespace

MotorBank::MotorBank(const BankConfig& config)
    : _config(config),
      _driver_A(config.a.pins),
      _driver_B(config.b.pins),
      _mutex(xSemaphoreCreateMutex())
{
    pinMode(_config.current_sense, INPUT);
    pinMode(_config.fault, INPUT_PULLUP);
}

MotorBank::~MotorBank() { vSemaphoreDelete(_mutex); }

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
    wake_control_task();
}

void MotorBank::set_target_position(const int16_t position)
{
    // Angles don't wrap and the bank won't drive past its limits, so a target
    // outside them could never be reached.
    const float degrees = position / kPositionUnitsPerDegree;
    if (degrees < _config.min_angle || degrees > _config.max_angle)
    {
        ESP_LOGW(TAG, "%s: SET_POSITION %.1f deg ignored: outside %.0f..%.0f", _config.name, degrees, _config.min_angle,
                 _config.max_angle);
        return;
    }
    {
        Lock lock(_mutex);
        _target_position = position;
        _mode = ControlMode::Position;
    }
    wake_control_task();
}

void MotorBank::stop()
{
    {
        Lock lock(_mutex);
        _speed = 0;
        _mode = ControlMode::Velocity;
    }
    wake_control_task();
}

void MotorBank::zero()
{
    if (_config.home_bite_current > 0.0f)
    {
        {
            Lock lock(_mutex);
            if (_mode == ControlMode::Homing)
                return;  // already running
            _speed = 0;
            _mode = ControlMode::Homing;
        }
        wake_control_task();
        return;
    }

    // A position target means something else once the angle is re-zeroed.
    stop();
    if (zero_encoders())
        ESP_LOGI(TAG, "%s: zero queued for both encoders", _config.name);
}

bool MotorBank::zero_encoders()
{
    const bool a = encoder_bus().zero(_config.a.encoder);
    const bool b = encoder_bus().zero(_config.b.encoder);
    if (!a || !b)
        ESP_LOGW(TAG, "%s: zero not queued, encoder bus not running or busy (encoder A %s, B %s)", _config.name,
                 a ? "ok" : "failed", b ? "ok" : "failed");
    return a && b;
}

// Falls back to the last angles read if neither encoder is fresh.
// TODO: that fallback hides a dead encoder from ROS; stop sending GET_POSITION instead.
int16_t MotorBank::get_current_position() const
{
    const uint32_t now = encoder_bus().now_ms();
    const std::optional<float> a = encoder_degrees(_config.a.encoder, now);
    const std::optional<float> b = encoder_degrees(_config.b.encoder, now);
    if (a || b)
    {
        const float degrees = (a && b) ? (*a + *b) / 2.0f : (a ? *a : *b);
        return static_cast<int16_t>(std::lround(degrees * kPositionUnitsPerDegree));
    }
    return static_cast<int16_t>((encoder_position(_config.a.encoder) + encoder_position(_config.b.encoder)) / 2);
}

MotorBank::Status MotorBank::get_status() const
{
    Lock lock(_mutex);
    return {_mode, _speed, _target_position, _output_a, _output_b};
}

float MotorBank::current_amps() const
{
    Lock lock(_mutex);
    return _current_amps;
}

bool MotorBank::is_in_fault() const { return digitalRead(_config.fault) == LOW; }

bool MotorBank::start_control_task(const std::array<MotorBank*, kBankCount>& banks)
{
    g_banks = banks;
    return xTaskCreatePinnedToCore(&MotorBank::control_task, "bank_control", kTaskStackBytes, nullptr, kTaskPriority,
                                   &g_task, kTaskCore) == pdPASS;
}

void MotorBank::control_task(void*)
{
    for (;;)
    {
        const uint32_t now_ms = encoder_bus().now_ms();
        for (MotorBank* bank : g_banks)
            if (bank)
                bank->control_tick(now_ms);
        // Returns early when wake_control_task() is called.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kControlPeriodMs));
    }
}

// Velocity mode applies the commanded speed, Position mode the control output,
// Homing mode the homing sequence.
void MotorBank::control_tick(uint32_t now_ms)
{
    const float amps = voltage_to_current(analogReadMilliVolts(_config.current_sense) / 1000.0f);

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
        if (_home_step == HomeStep::Idle)
        {
            _home_bites = 0;
            set_home_step(HomeStep::Open, now_ms);
            ESP_LOGI(TAG, "%s homing: opening until the current falls below %.2f A", _config.name,
                     _config.home_idle_current);
        }
        output_a = output_b = homing_output(now_ms, amps);
    }
    else
    {
        if (_home_step != HomeStep::Idle)
        {
            _home_step = HomeStep::Idle;
            ESP_LOGW(TAG, "%s homing: cancelled by a command", _config.name);
        }

        // Both actuators move the same joint, so a side with no reading
        // follows the other side's encoder.
        const std::optional<float> own_a = encoder_degrees(_config.a.encoder, now_ms);
        const std::optional<float> own_b = encoder_degrees(_config.b.encoder, now_ms);
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
                _config.speed_direction < 0 ? static_cast<int16_t>(-std::max<int32_t>(speed, -INT16_MAX)) : speed;
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
    const bool raising = (output > 0) == (_config.position_direction > 0);
    const bool blocked = raising ? *angle >= _config.max_angle : *angle <= _config.min_angle;
    return blocked ? 0 : output;
}

// One step of the homing sequence; returns the duty for both motors.
int16_t MotorBank::homing_output(uint32_t now_ms, float amps)
{
    const int16_t open = to_duty(kHomeSpeed) * _config.position_direction;  // opening raises the angle
    const int16_t close = -open;
    const uint32_t elapsed = now_ms - _home_step_start_ms;

    if (is_in_fault())
        return end_homing("driver fault");

    switch (_home_step)
    {
    case HomeStep::Idle:
        break;

    case HomeStep::Open:
        if (home_detect(amps <= _config.home_idle_current || amps >= _config.home_bite_current, now_ms))
        {
            // A stall here means the jaws ran into something, or are closing instead.
            if (amps >= _config.home_bite_current)
                return end_homing("stalled while opening");
            ESP_LOGI(TAG, "%s homing: open end reached (%.2f A)", _config.name, amps);
            set_home_step(HomeStep::Bite, now_ms);
            return close;
        }
        if (elapsed >= kHomeTimeoutMs)
            return end_homing("current never fell to idle while opening");
        return open;

    case HomeStep::Bite:
        if (home_detect(amps >= _config.home_bite_current, now_ms))
        {
            ++_home_bites;
            ESP_LOGI(TAG, "%s homing: bite %u/%u at %.2f A", _config.name, _home_bites, kHomeBites, amps);
            if (_home_bites < kHomeBites)
            {
                set_home_step(HomeStep::Backoff, now_ms);
                return open;
            }
            if (zero_encoders())
                ESP_LOGI(TAG, "%s homing: encoders zeroed at the clench", _config.name);
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
    _home_detect_since.reset();
}

// True once `condition` has held for kHomeDetectMs, not counting the inrush
// at the start of the step.
bool MotorBank::home_detect(bool condition, uint32_t now_ms)
{
    if (!condition || now_ms - _home_step_start_ms < kHomeInrushMs)
        _home_detect_since.reset();
    else if (!_home_detect_since)
        _home_detect_since = now_ms;
    return _home_detect_since && now_ms - *_home_detect_since >= kHomeDetectMs;
}

// Stops the motors and hands back to Velocity mode, unless a command already
// took over.
int16_t MotorBank::end_homing(const char* abort_reason)
{
    _home_step = HomeStep::Idle;
    {
        Lock lock(_mutex);
        if (_mode == ControlMode::Homing)
        {
            _speed = 0;
            _mode = ControlMode::Velocity;
        }
    }
    if (abort_reason)
        ESP_LOGW(TAG, "%s homing: aborted, %s", _config.name, abort_reason);
    else
        ESP_LOGI(TAG, "%s homing: done", _config.name);
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

    const int16_t duty = to_duty(kPositionSpeed) * _config.position_direction;
    return error > 0 ? duty : -duty;
}

std::optional<float> MotorBank::encoder_degrees(EncoderId id, uint32_t now_ms)
{
    EncoderReading reading;
    if (!encoder_bus().get(id, &reading) || reading.angle_age_ms(now_ms) > kFeedbackStaleMs)
        return std::nullopt;
    return reading.degrees;
}

int16_t MotorBank::encoder_position(EncoderId id)
{
    EncoderReading reading;
    encoder_bus().get(id, &reading);
    return static_cast<int16_t>(std::lround(reading.degrees * kPositionUnitsPerDegree));
}
