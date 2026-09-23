#pragma once

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Minimal PID for one bank's closed-loop position hold. dt is fixed at
// construction (matches the control task's period) and is never carried on
// the wire - only Kp/Ki/Kd are CAN-tunable, via bank_parameter::SET_PID_PARAMS.
class PidController
{
public:
    struct Gains
    {
        double kp = 0.0;
        double ki = 0.0;
        double kd = 0.0;
    };

    explicit PidController(double dt_seconds);
    ~PidController();

    PidController(const PidController&) = delete;
    PidController& operator=(const PidController&) = delete;

    // Thread-safe: called from the CAN callback thread.
    void set_gains(const Gains& gains);
    Gains get_gains() const;

    // Control-task-only. Not synchronized against each other or against
    // set_gains()'s snapshot read - MotorBank's own mutex already serializes
    // all control-tick access, this class only protects the gains themselves.
    void reset();
    int16_t compute(int16_t setpoint, int16_t measurement);

private:
    const double _dt;
    Gains _gains{};
    double _integral = 0.0;
    double _prev_error = 0.0;
    SemaphoreHandle_t _gains_mutex;
};
