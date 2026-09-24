#include "motor_driver.hpp"

#include <Arduino.h>  // TODO: required to load before motor_driver.hpp due to IPADDR_NONE collission

#include <stdexcept>

#include "shared_memory.hpp"

static constexpr uint8_t PWM_BITS = 12;
static constexpr uint32_t PWM_FREQ = 1500;  // Hz

// 2 PWM steps of enforced deadband to reset cycle-by-cycle current chopping
static constexpr uint32_t PWM_DEADBAND = 2;

static constexpr uint32_t PWM_MAX = (1 << PWM_BITS) - 1 - PWM_DEADBAND;  // 4095

MotorDriver::MotorDriver(const bsp::pin_pair_t& pins, EncoderId encoder_id, uint8_t encoder_group_id, EncoderBus* encoder_bus)
    : _pins(pins),
      _motor_memory(encoder_id, encoder_group_id, encoder_bus)  // Replace 0, 0 with appropriate encoder_id and encoder_group_id if available
{
    pinMode(pins.first, OUTPUT);
    pinMode(pins.second, OUTPUT);
}

MotorDriver::~MotorDriver()
{
    digitalWrite(_pins.first, LOW);
    digitalWrite(_pins.second, LOW);
    pinMode(_pins.first, INPUT);
    pinMode(_pins.second, INPUT);
}

void MotorDriver::set_speed(int16_t speed)
{
    _motor_memory.set_speed(speed);
}

void MotorDriver::set_target_position(int16_t position)
{
    _motor_memory.set_target_position(position);
}

int16_t MotorDriver::get_current_position() const
{
    return _motor_memory.get_current_position();
}

// a single pass of checking motor status and updating PWM value accordingly
void MotorDriver::monitor_and_move(void)
{
    _motor_memory.monitor_and_move();  // updates the memory

    // TODO: Implement movement logic here
    // if (get_current_position() != _motor_memory.get_target_position())
}

// void MotorDriver::set_speed(int16_t speed)
// {
//     speed = map(speed, std::numeric_limits<int16_t>::min(),
//                 std::numeric_limits<int16_t>::max(), -PWM_MAX, PWM_MAX);

//     direction current_dir = direction::STOPPED;
//     if (speed > 0)
//         current_dir = direction::FORWARD;
//     else if (speed < 0)
//         current_dir = direction::BACKWARD;
//     bool dir_changed = (current_dir != _prev_direction);
//     _prev_direction = current_dir;

//     if (dir_changed)
//     {
//         if (current_dir == direction::FORWARD)
//         {
//             pinMode(_pins.second, OUTPUT);
//             digitalWrite(_pins.second, LOW);
//             analogWrite(_pins.first, 1);
//             analogWriteResolution(_pins.first, PWM_BITS);
//             analogWriteFrequency(_pins.first, PWM_FREQ);
//         }
//         else if (current_dir == direction::BACKWARD)
//         {
//             pinMode(_pins.first, OUTPUT);
//             digitalWrite(_pins.first, LOW);
//             analogWrite(_pins.second, 1);
//             analogWriteResolution(_pins.second, PWM_BITS);
//             analogWriteFrequency(_pins.second, PWM_FREQ);
//         }
//         else
//         {
//             pinMode(_pins.first, OUTPUT);
//             pinMode(_pins.second, OUTPUT);
//             digitalWrite(_pins.first, LOW);
//             digitalWrite(_pins.second, LOW);
//         }
//     }

//     if (speed > 0)
//         analogWrite(_pins.first, speed);
//     else if (speed < 0)
//         analogWrite(_pins.second, -speed);
// }
