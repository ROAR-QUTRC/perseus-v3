#include "pid_controller.hpp"

#include <algorithm>
#include <limits>

namespace
{
    constexpr double kOutputMin = std::numeric_limits<int16_t>::min();
    constexpr double kOutputMax = std::numeric_limits<int16_t>::max();

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
}  // namespace

PidController::PidController(double dt_seconds)
    : _dt(dt_seconds),
      _gains_mutex(xSemaphoreCreateMutex())
{
}

PidController::~PidController()
{
    vSemaphoreDelete(_gains_mutex);
}

void PidController::set_gains(const Gains& gains)
{
    Lock lock(_gains_mutex);
    _gains = gains;
}

PidController::Gains PidController::get_gains() const
{
    Lock lock(_gains_mutex);
    return _gains;
}

void PidController::reset()
{
    _integral = 0.0;
    _prev_error = 0.0;
}

int16_t PidController::compute(int16_t setpoint, int16_t measurement)
{
    const Gains gains = get_gains();  // one locked snapshot per tick
    const double error = static_cast<double>(setpoint) - static_cast<double>(measurement);
    const double proposed_integral = _integral + error * _dt;
    const double derivative = (error - _prev_error) / _dt;

    const double output = gains.kp * error + gains.ki * proposed_integral + gains.kd * derivative;

    // Clamped-integrator anti-windup: only accumulate while not saturated.
    if (output <= kOutputMax && output >= kOutputMin)
        _integral = proposed_integral;

    _prev_error = error;
    return static_cast<int16_t>(std::clamp(output, kOutputMin, kOutputMax));
}
