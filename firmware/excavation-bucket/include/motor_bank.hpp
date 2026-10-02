#pragma once

#include <driver/sdm.h>

#include <board_support.hpp>
#include <cstdint>
#include <optional>

#include "encoder_bus.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "hi_can_packet.hpp"
#include "motor_driver.hpp"
#include "shared_memory.hpp"

/**
 * @brief A class responsible for managing pair of motors (Left = A, Right = B) (TODO: check this and update)
 * @details Both motors always get the same command: SET_SPEED (velocity) or SET_POSITION (position) is per bank.
 */
class MotorBank : public ExcavationJoint
{
public:
    static constexpr uint16_t CURRENT_SENSE_RESISTOR = 1000;        // ohms
    static constexpr float CURRENT_SENSE_PROPORTIONALITY = 450e-6;  // amps per amp

    static constexpr float voltage_to_current(const float& voltage)
    {
        return voltage / (CURRENT_SENSE_RESISTOR * CURRENT_SENSE_PROPORTIONALITY);
    }

    static constexpr float MAX_VOLTAGE = 3.3f;  // volts
    static constexpr float MAX_CURRENT = 6.0f;  // amps

    // Longest gap between control_tick() calls; a new command runs one straight away.
    static constexpr uint32_t kControlPeriodMs = 20;

    // Encoder readings older than this are treated as missing. 5x EncoderBus::kAnglePeriodMs.
    static constexpr uint32_t kFeedbackStaleMs = 100;

    // SET_SPEED values within this band don't leave Position mode, so teleop's
    // stream of zeros and the SET_SPEED timeout can't cancel a setpoint.
    // Speeds are % duty cycle (100 = full PWM); angles and windows are degrees.
    static constexpr float kSpeedDeadband = 1.0f;

    // Position control: constant speed toward the target, stop within kHoldWindow.
    static constexpr float kPositionSpeed = 100.0f;  // the actuators stall at ~10%
    static constexpr float kHoldWindow = 2.0f;
    static constexpr float kResumeWindow = 3.0f;   // once stopped, restart only past this, so noise can't chatter
    static constexpr float kMaxAngle = 180.0f;     // angles and targets are -180..180, no wrap-around

    // Homing: open until the current falls to idle (actuators on their end
    // switches), then bite and back off kHomeBites times. The encoders are
    // zeroed during the last bite. Timed and current-based only; it never
    // reads the encoder angle, and the angle limits don't apply to it.
    static constexpr float kHomeSpeed = 100.0f;
    static constexpr uint8_t kHomeBites = 3;
    static constexpr uint32_t kHomeBackoffMs = 500;
    static constexpr uint32_t kHomeInrushMs = 200;      // current ignored after each start or reversal
    static constexpr uint32_t kHomeDetectMs = 100;      // current must stay past a threshold this long
    static constexpr uint32_t kHomeZeroHoldMs = 300;    // clench held while the encoders take the zero
    static constexpr uint32_t kHomeTimeoutMs = 30'000;  // per open or bite move

    // SET_SPEED and MotorDriver::drive() use int16 duty, +/-32767 = 100%.
    static constexpr int16_t to_duty(float percent) { return static_cast<int16_t>(percent * 32767.0f / 100.0f); }
    static constexpr float to_percent(int16_t duty) { return duty * 100.0f / 32767.0f; }

    enum class ControlMode : uint8_t
    {
        Velocity,  // SET_SPEED: the commanded speed goes straight to the motors
        Position,  // SET_POSITION: control_tick() drives toward the target
        Homing,    // start_homing(): control_tick() runs the homing sequence
    };

    struct Status
    {
        ControlMode mode;
        int16_t speed;
        int16_t target_position;  // position_t units (degrees x10)
        int16_t output_a;         // last values sent to the drivers
        int16_t output_b;
    };

    MotorBank(const bsp::pin_pair_t& driver_A_pins, EncoderId driver_A_encoder_id, uint8_t driver_A_encoder_group_id,
              const bsp::pin_pair_t& driver_B_pins, EncoderId driver_B_encoder_id, uint8_t driver_B_encoder_group_id,
              const gpio_num_t& current_sense_pin, const gpio_num_t& fault_pin,
              int8_t speed_direction, int8_t position_direction, float min_angle, float max_angle,
              EncoderBus* encoder_bus);

    // delete copy/move semantics
    MotorBank(const MotorBank&) = delete;
    MotorBank(MotorBank&&) = delete;
    MotorBank& operator=(const MotorBank&) = delete;
    MotorBank& operator=(MotorBank&&) = delete;

    virtual ~MotorBank();

    // Caches each side's encoder angle for the per-encoder GET_ANGLE reports.
    void monitor_and_move(void) override;

    // SET_SPEED: velocity command. A speed outside kSpeedDeadband switches to Velocity mode.
    void set_speed(const int16_t speed) override;
    // SET_POSITION: position command, in position_t units. Switches to Position mode.
    // A target outside the bank's angle limits is ignored.
    void set_target_position(const int16_t position) override;
    // Speed 0 in Velocity mode, which a SET_SPEED of 0 alone can't force. The
    // next SET_POSITION re-arms position control.
    void stop();

    // Call before the control task starts. Currents are amps, as GET_CURRENT reports them.
    void enable_homing(float bite_current, float idle_current);
    bool homing_enabled() const { return _home_bite_current > 0.0f; }
    // Any SET_SPEED outside the deadband, SET_POSITION or stop() cancels it.
    void start_homing();
    // GET_POSITION: average of the bank's encoders, in position_t units.
    int16_t get_current_position() const override;
    Status get_status() const;

    int16_t get_current_position_a() const;
    int16_t get_current_position_b() const;
    MotorDriver& get_driver_A(void);
    MotorDriver& get_driver_B(void);

    std::vector<uint8_t> get_current();
    // Amps, as sampled by the last control_tick().
    float get_average_current();
    bool is_in_fault();
    std::vector<uint8_t> get_fault();

    // Called by bank_control_task every kControlPeriodMs and after each command. The only place the
    // motors are driven: Velocity mode applies the commanded speed, Position
    // mode the control output, Homing mode the homing sequence. Also the only
    // place the current is sampled: the ADC read fails if two tasks overlap.
    void control_tick(uint32_t now_ms);

    // Degrees (-180..180) of any encoder, or nullopt if it has no valid angle or
    // it is older than kFeedbackStaleMs.
    static std::optional<float> encoder_degrees(EncoderId id, uint32_t now_ms);

private:
    enum class HomeStep : uint8_t
    {
        Open,     // to the open end, until the current falls to idle
        Bite,     // close until the current passes the bite threshold
        Zero,     // last bite: hold the clench while the encoders zero
        Backoff,  // open for kHomeBackoffMs
    };

    int16_t position_output(float target, std::optional<float> angle, bool* settled) const;
    int16_t limit_output(int16_t output, std::optional<float> angle) const;
    int16_t homing_output(uint32_t now_ms, float amps);
    void set_home_step(HomeStep step, uint32_t now_ms);
    bool home_detect(bool condition, uint32_t now_ms);
    int16_t end_homing(const char* abort_reason = nullptr);

    MotorDriver _driver_A;
    MotorDriver _driver_B;

    const gpio_num_t _current_sense_pin;
    const gpio_num_t _fault_pin;

    // From excavation_config.hpp.
    const int8_t _speed_direction;     // 1 or -1, applied to SET_SPEED
    const int8_t _position_direction;  // 1 or -1: the motor sign that raises the angle
    const float _min_angle;
    const float _max_angle;
    float _home_bite_current = 0.0f;  // 0 = this bank doesn't home
    float _home_idle_current = 0.0f;

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
    bool _home_active = false;
    HomeStep _home_step = HomeStep::Open;
    uint8_t _home_bites = 0;
    uint32_t _home_step_start_ms = 0;
    bool _home_detecting = false;
    uint32_t _home_detect_start_ms = 0;
};
