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

    // Period bank_control_task calls control_tick() at.
    static constexpr uint32_t kControlPeriodMs = 20;

    // Encoder readings older than this are treated as missing. 5x EncoderBus::kAnglePeriodMs.
    static constexpr uint32_t kFeedbackStaleMs = 100;

    // SET_SPEED values within this band don't leave Position mode, so teleop's
    // stream of zeros and the SET_SPEED timeout can't cancel a setpoint.
    static constexpr int16_t kSpeedDeadband = 327;  // ~1% of full scale

    // Position control: constant speed toward the target, stop within kHoldWindow (degrees).
    static constexpr int16_t kPositionSpeed = 16384;  // ~50%; the actuators stall at ~10%
    static constexpr float kHoldWindow = 2.0f;
    static constexpr float kResumeWindow = 3.0f;  // once stopped, restart only past this, so noise can't chatter
    static constexpr int8_t kDriveDirection = 1;  // set -1 if positive speed decreases the angle

    enum class ControlMode : uint8_t
    {
        Velocity,  // SET_SPEED: the commanded speed goes straight to the motors
        Position,  // SET_POSITION: control_tick() drives toward the target
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
              const gpio_num_t& current_sense_pin, const gpio_num_t& fault_pin, EncoderBus* encoder_bus);

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
    void set_target_position(const int16_t position) override;
    // Speed 0 in Velocity mode, which a SET_SPEED of 0 alone can't force. The
    // next SET_POSITION re-arms position control.
    void stop();
    // GET_POSITION: average of the bank's encoders, in position_t units.
    int16_t get_current_position() const override;
    Status get_status() const;

    int16_t get_current_position_a() const;
    int16_t get_current_position_b() const;
    MotorDriver& get_driver_A(void);
    MotorDriver& get_driver_B(void);

    std::vector<uint8_t> get_current();
    float get_average_current();
    bool is_in_fault();
    std::vector<uint8_t> get_fault();

    // Called every kControlPeriodMs by bank_control_task. The only place the
    // motors are driven: Velocity mode applies the commanded speed, Position
    // mode the control output.
    void control_tick(uint32_t now_ms);

    // Degrees (0-360) of any encoder, or nullopt if it has no valid angle or
    // it is older than kFeedbackStaleMs.
    static std::optional<float> encoder_degrees(EncoderId id, uint32_t now_ms);

    // Shortest signed angle from `from` to `to`, in (-180, 180]: 359 to 1 is +2.
    static float angle_difference(float to, float from);

private:
    static int16_t position_output(float target, std::optional<float> angle, bool* settled);

    MotorDriver _driver_A;
    MotorDriver _driver_B;

    const gpio_num_t _current_sense_pin;
    const gpio_num_t _fault_pin;

    // Written by the CAN task, read by the control task; guarded by _mutex.
    SemaphoreHandle_t _mutex;
    ControlMode _mode = ControlMode::Velocity;
    int16_t _speed = 0;
    int16_t _target_position = 0;
    int16_t _output_a = 0;
    int16_t _output_b = 0;

    // Owned by the control task.
    ControlMode _last_mode = ControlMode::Velocity;
    int16_t _last_target = 0;
    bool _settled_a = false;
    bool _settled_b = false;
};
