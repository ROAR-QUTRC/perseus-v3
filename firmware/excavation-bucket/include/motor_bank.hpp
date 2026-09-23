#pragma once

#include <driver/sdm.h>

#include <board_support.hpp>
#include <cstdint>

#include "encoder_bus.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "hi_can_packet.hpp"
#include "motor_driver.hpp"
#include "pid_controller.hpp"

class MotorBank
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

    // Fixed period the bank control task calls control_tick() at; also
    // PidController's dt. Must match bank_control_task's own period.
    static constexpr uint32_t kControlPeriodMs = 20;

    // A feedback reading older than this (both sides) is treated as a sensor
    // fault by control_tick(): drop back to open loop and stop, rather than
    // drive on stale data. 5x EncoderBus::kAnglePeriodMs.
    static constexpr uint32_t kFeedbackStaleMs = 100;

    MotorBank(const bsp::pin_pair_t& driver_A_pins,
              const bsp::pin_pair_t& driver_B_pins,
              const gpio_num_t& current_sense_pin, const gpio_num_t& fault_pin,
              EncoderId encoder_left, EncoderId encoder_right);

    // delete copy/move semantics
    MotorBank(const MotorBank&) = delete;
    MotorBank(MotorBank&&) = delete;
    MotorBank& operator=(const MotorBank&) = delete;
    MotorBank& operator=(MotorBank&&) = delete;

    virtual ~MotorBank();

    // Open-loop: forces the bank out of closed-loop position hold (if active)
    // and drives both actuators directly. Always wins immediately - this is
    // the same fail-safe path both CAN watchdogs (SET_SPEED, SET_POSITION)
    // and control_tick()'s own feedback-fault handling converge on.
    void set_speed(const int16_t speed);
    void set_speed_a(const int16_t speed) { _driver_A.set_speed(speed); }
    void set_speed_b(const int16_t speed) { _driver_B.set_speed(speed); }

    // Closed-loop: switches the bank into position hold (resetting the PID's
    // integrator on entry) and stores the target. Actual motor output only
    // ever happens inside control_tick().
    void set_position_setpoint(int16_t setpoint);
    void set_pid_gains(const PidController::Gains& gains);

    // Runs one control-loop iteration; a no-op unless currently in
    // closed-loop position mode. Must only be called from the bank control
    // task (see bank_control_task.hpp).
    void control_tick(uint32_t now_ms);

    std::vector<uint8_t> get_current();
    float get_average_current();
    bool is_in_fault();
    std::vector<uint8_t> get_fault();

    // Averaged raw encoder counts across the bank's two sides, serialized as
    // position_t. Reports 0 if neither side currently has fresh, valid data.
    std::vector<uint8_t> get_position();

private:
    enum class ControlMode : uint8_t
    {
        OPEN_LOOP_SPEED,
        CLOSED_LOOP_POSITION,
    };

    class Lock
    {
    public:
        explicit Lock(SemaphoreHandle_t mutex)
            : _mutex(mutex)
        {
            xSemaphoreTake(_mutex, portMAX_DELAY);
        }
        ~Lock() { xSemaphoreGive(_mutex); }

        Lock(const Lock&) = delete;
        Lock& operator=(const Lock&) = delete;

    private:
        SemaphoreHandle_t _mutex;
    };

    // Averages whichever of the bank's two encoders currently has fresh,
    // valid data. Returns false (leaving *out untouched) if neither does.
    bool average_raw_counts(uint32_t now_ms, int16_t* out) const;

    MotorDriver _driver_A;
    MotorDriver _driver_B;

    const gpio_num_t _current_sense_pin;
    const gpio_num_t _fault_pin;

    const EncoderId _encoder_left;
    const EncoderId _encoder_right;

    PidController _position_pid;
    ControlMode _mode = ControlMode::OPEN_LOOP_SPEED;
    int16_t _position_setpoint = 0;

    // Guards _mode, _position_setpoint, and every _driver_A/_driver_B write -
    // the state/hardware touched by both the CAN callback thread (set_speed(),
    // set_position_setpoint()) and the control task (control_tick()).
    SemaphoreHandle_t _mutex;
};
