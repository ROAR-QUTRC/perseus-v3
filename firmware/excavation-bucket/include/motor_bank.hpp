#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "encoder_bus.hpp"
#include "excavation_config.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "motor_driver.hpp"

// position_t on CAN (GET_ANGLE, GET_POSITION, SET_POSITION) is degrees x10.
inline constexpr float kPositionUnitsPerDegree = 10.0f;

/**
 * @brief A class responsible for managing pair of motors (Left = A, Right = B) (TODO: check this and update)
 * @details Both motors always get the same command: SET_SPEED (velocity) or SET_POSITION (position) is per bank.
 */
class MotorBank
{
public:
    static constexpr uint16_t CURRENT_SENSE_RESISTOR = 1000;        // ohms
    static constexpr float CURRENT_SENSE_PROPORTIONALITY = 450e-6;  // amps per amp

    static constexpr float voltage_to_current(const float& voltage)
    {
        return voltage / (CURRENT_SENSE_RESISTOR * CURRENT_SENSE_PROPORTIONALITY);
    }

    // Longest gap between control ticks; a new command runs one straight away.
    // No point ticking faster than the encoder data refreshes.
    static constexpr uint32_t kControlPeriodMs = EncoderBus::kAnglePeriodMs;

    // Encoder readings older than this are treated as missing.
    static constexpr uint32_t kFeedbackStaleMs = 5 * EncoderBus::kAnglePeriodMs;

    // SET_SPEED values within this band don't leave Position mode, so teleop's
    // stream of zeros and the SET_SPEED timeout can't cancel a setpoint.
    // Speeds are % duty cycle (100 = full PWM); angles and windows are degrees.
    static constexpr float kSpeedDeadband = 1.0f;

    // Position control: constant speed toward the target, stop within kHoldWindow.
    static constexpr float kPositionSpeed = 100.0f;  // the actuators stall at ~10%
    static constexpr float kHoldWindow = 2.0f;
    static constexpr float kResumeWindow = 3.0f;  // once stopped, restart only past this, so noise can't chatter
    static constexpr float kMaxAngle = 180.0f;    // angles and targets are -180..180, no wrap-around

    // SET_SPEED and MotorDriver::drive() use int16 duty, +/-32767 = 100%.
    static constexpr int16_t to_duty(float percent) { return static_cast<int16_t>(percent * 32767.0f / 100.0f); }
    static constexpr float to_percent(int16_t duty) { return duty * 100.0f / 32767.0f; }

    enum class ControlMode : uint8_t
    {
        Velocity,  // SET_SPEED: the commanded speed goes straight to the motors
        Position,  // SET_POSITION: the control task drives toward the target
        Homing,    // SET_ZERO_POS on a bank with a homing routine: the control task runs it
    };

    struct Status
    {
        ControlMode mode;
        int16_t speed;
        int16_t target_position;  // position_t units (degrees x10)
        int16_t output_a;         // last values sent to the drivers
        int16_t output_b;
    };

    explicit MotorBank(const BankConfig& config);

    // delete copy/move semantics
    MotorBank(const MotorBank&) = delete;
    MotorBank(MotorBank&&) = delete;
    MotorBank& operator=(const MotorBank&) = delete;
    MotorBank& operator=(MotorBank&&) = delete;

    ~MotorBank();

    // SET_SPEED: velocity command. A speed outside kSpeedDeadband switches to Velocity mode.
    void set_speed(int16_t speed);
    // SET_POSITION: position command, in position_t units. Switches to Position mode.
    // A target outside the bank's angle limits is ignored.
    void set_target_position(int16_t position);
    // Speed 0 in Velocity mode, which a SET_SPEED of 0 alone can't force. The
    // next SET_POSITION re-arms position control.
    void stop();
    // SET_ZERO_POS: stops and zeroes both encoders where they are, each saving
    // its new offset to flash. A bank with a homing routine runs that instead,
    // which picks the moment to zero; any SET_SPEED outside the deadband,
    // SET_POSITION or stop() cancels it, and so does a driver fault or the failsafe.
    void zero();

    // GET_POSITION: average of the bank's encoders, in position_t units.
    int16_t get_current_position() const;
    // GET_CURRENT: amps, as sampled by the last control tick.
    float current_amps() const;
    bool is_in_fault() const;
    Status get_status() const;

    // Starts the FreeRTOS task that runs every bank's control tick each
    // kControlPeriodMs, or straight away after a command. It is the only place
    // the motors are driven, and the only place the current is sampled: the
    // ADC read fails if two tasks overlap. Every tick it asks bucket_may_run()
    // and holds the motors stopped while that is false. Call once from setup(), after
    // encoder_bus().begin(). Pinned to core 0 alongside EncoderBus's RS485
    // tasks; loop() (CAN handling) stays on core 1.
    static bool start_control_task(const std::array<MotorBank*, kBankCount>& banks);
    // Runs the next control tick now instead of waiting out the period. No-op
    // before the task has started.
    static void wake_control_task();

    // Degrees (-180..180) of any encoder, or nullopt if it has no valid angle or
    // it is older than kFeedbackStaleMs.
    static std::optional<float> encoder_degrees(EncoderId id, uint32_t now_ms);
    // GET_ANGLE: the last angle read from an encoder in position_t units,
    // however old; 0 before the first read.
    static int16_t encoder_position(EncoderId id);

private:
    static void control_task(void*);
    void control_tick(uint32_t now_ms, bool may_run);
    int16_t position_output(float target, std::optional<float> angle, bool* settled) const;
    int16_t limit_output(int16_t output, std::optional<float> angle) const;
    int16_t homing_output(uint32_t now_ms, float amps, bool may_run);
    bool zero_encoders();

    const BankConfig& _config;
    MotorDriver _driver_A;
    MotorDriver _driver_B;

    // Written by the CAN task, read by the control task; guarded by _mutex.
    SemaphoreHandle_t _mutex;
    ControlMode _mode = ControlMode::Velocity;
    int16_t _speed = 0;
    int16_t _target_position = 0;
    int16_t _output_a = 0;
    int16_t _output_b = 0;
    float _current_amps = 0.0f;

    // Owned by the control task.
    ControlMode _last_mode = ControlMode::Velocity;
    int16_t _last_target = 0;
    bool _settled_a = false;
    bool _settled_b = false;
    std::optional<HomingState> _homing;  // set while the homing routine runs
};
