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

    constexpr bool config_valid(const BankConfig& bank)
    {
        const auto is_direction = [](int8_t direction)
        { return direction == 1 || direction == -1; };
        return is_direction(bank.speed_direction) && is_direction(bank.position_direction) &&
               bank.min_angle >= -MotorBank::kMaxAngle && bank.max_angle <= MotorBank::kMaxAngle &&
               bank.min_angle < bank.max_angle;
    }
    static_assert(std::ranges::all_of(kBanks, [](const BankConfig* bank)
                                      { return config_valid(*bank); }),
                  "excavation_config.hpp: each direction must be 1 or -1, and min_angle < max_angle within -180..180");
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
    if (_config.homing)
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

void MotorBank::wake_control_task()
{
    if (g_task)
        xTaskNotifyGive(g_task);
}

void MotorBank::control_task(void*)
{
    for (;;)
    {
        const uint32_t now_ms = encoder_bus().now_ms();
        const bool may_run = bucket_may_run();
        for (MotorBank* bank : g_banks)
            if (bank)
                bank->control_tick(now_ms, may_run);
        // Returns early when wake_control_task() is called.
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kControlPeriodMs));
    }
}

// Velocity mode applies the commanded speed, Position mode the control output,
// Homing mode the homing routine. In failsafe (!may_run) the motors get 0 but
// the mode, speed and target stay, so the bank carries on when it clears.
void MotorBank::control_tick(uint32_t now_ms, bool may_run)
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
        output_a = output_b = homing_output(now_ms, amps, may_run);
    }
    else
    {
        if (_homing)
        {
            _homing.reset();
            ESP_LOGW(TAG, "%s: homing cancelled by a command", _config.name);
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

    if (!may_run)
        output_a = output_b = 0;

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

// One tick of the bank's homing routine; returns the duty for both motors.
int16_t MotorBank::homing_output(uint32_t now_ms, float amps, bool may_run)
{
    if (!_homing)
    {
        _homing = HomingState{.since = now_ms};
        ESP_LOGI(TAG, "%s: homing", _config.name);
    }

    // A routine can't sit out a stop: it reads the motors' current.
    HomingStep step{.done = true};
    if (!may_run || is_in_fault())
        ESP_LOGW(TAG, "%s: homing aborted, %s", _config.name, may_run ? "driver fault" : "failsafe");
    else
        step = _config.homing(*_homing, now_ms, amps);

    if (step.zero && zero_encoders())
        ESP_LOGI(TAG, "%s: encoders zeroed by homing", _config.name);
    if (step.done)
    {
        // Back to Velocity mode at speed 0, unless a command already took over.
        _homing.reset();
        Lock lock(_mutex);
        if (_mode == ControlMode::Homing)
        {
            _speed = 0;
            _mode = ControlMode::Velocity;
        }
    }
    return to_duty(step.duty) * _config.position_direction;
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
