#pragma once

#include "board_support.hpp"
#include "encoder_bus.hpp"

using namespace bsp;

// Note:
// We're only using the analog functions on the current sense pins,
// so these are the only ones named with analog numbers
// even though all the pins can do analog and digital IO

// Encoders: each driver's ENCODER_BUS and ENCODER_SLAVE say which encoder board it
// reads. ENCODER_SLAVE is the Modbus address, i.e. the board's DIP value + 1.
// Left encoders are on bus 1 and right on bus 2, so one cable fault leaves each
// joint with a working encoder. Checked at compile time in encoder_bus.cpp.

// Directions: 1 or -1 per bank, and independent of each other.
// SPEED_DIRECTION: flip if a positive SET_SPEED moves the joint the wrong way.
// POSITION_DIRECTION: flip if SET_POSITION drives away from the target, i.e.
// use -1 when driving the motors forward makes the encoder angle go down.

// Limits: a bank won't drive below MIN_ANGLE or above MAX_ANGLE (degrees). Lift
// and tilt have none yet, so theirs are the full range.

// Homing (jaws only, started by SET_ZERO_POS): opens until the current falls
// below HOME_IDLE_CURRENT, then bites until it passes HOME_BITE_CURRENT and
// backs off, a few times. The encoders are zeroed during the last bite, so 0
// degrees is the clenched frame. Amps, as GET_CURRENT reports them. Opening
// must raise the angle. Timing is in motor_bank.hpp (kHome...).

namespace LIFT
{
    // TODO update namespace to be LEFT and RIGHT based (once pins are confirmed)
    namespace DRIVER_A
    {                                                                       // assuming left
        static constexpr pin_pair_t DRIVER_PINS{GPIO_NUM_16, GPIO_NUM_15};  // driver 1
        static constexpr EncoderId ENCODER_ID = EncoderId::LiftLeft;        // for reading from bus
        static constexpr uint8_t ENCODER_BUS = 1;                           // RS485 bus, 1 or 2
        static constexpr uint8_t ENCODER_SLAVE = 1;                         // Modbus address = DIP value + 1
        static constexpr uint8_t GROUP_ID = 0x03;                           // for sending over canbus
    }
    namespace DRIVER_B
    {                                                                       // assuming Right
        static constexpr pin_pair_t DRIVER_PINS{GPIO_NUM_41, GPIO_NUM_42};  // driver 2
        static constexpr EncoderId ENCODER_ID = EncoderId::LiftRight;       // for reading from bus
        static constexpr uint8_t ENCODER_BUS = 2;                           // RS485 bus, 1 or 2
        static constexpr uint8_t ENCODER_SLAVE = 1;                         // Modbus address = DIP value + 1
        static constexpr uint8_t GROUP_ID = 0x04;                           // for sending over canbus
    }
    static constexpr gpio_num_t CURRENT_SENSE = bsp::A1;
    static constexpr gpio_num_t FAULT = GPIO_NUM_2;
    static constexpr int8_t SPEED_DIRECTION = 1;
    static constexpr int8_t POSITION_DIRECTION = -1;
    static constexpr float MIN_ANGLE = -180.0f;
    static constexpr float MAX_ANGLE = 180.0f;
}

namespace TILT
{
    namespace DRIVER_A
    {                                                                       // assuming left
        static constexpr pin_pair_t DRIVER_PINS{GPIO_NUM_38, GPIO_NUM_37};  // driver 3
        static constexpr EncoderId ENCODER_ID = EncoderId::TiltLeft;        // for reading from bus
        static constexpr uint8_t ENCODER_BUS = 1;                           // RS485 bus, 1 or 2
        static constexpr uint8_t ENCODER_SLAVE = 2;                         // Modbus address = DIP value + 1
        static constexpr uint8_t GROUP_ID = 0x05;                           // for sending over canbus
    }
    namespace DRIVER_B
    {                                                                       // assuming Right
        static constexpr pin_pair_t DRIVER_PINS{GPIO_NUM_45, GPIO_NUM_48};  // driver 4
        static constexpr EncoderId ENCODER_ID = EncoderId::TiltRight;       // for reading from bus
        static constexpr uint8_t ENCODER_BUS = 2;                           // RS485 bus, 1 or 2
        static constexpr uint8_t ENCODER_SLAVE = 2;                         // Modbus address = DIP value + 1
        static constexpr uint8_t GROUP_ID = 0x06;                           // for sending over canbus
    }
    static constexpr gpio_num_t CURRENT_SENSE = bsp::A3;
    static constexpr gpio_num_t FAULT = GPIO_NUM_4;
    static constexpr int8_t SPEED_DIRECTION = 1;
    static constexpr int8_t POSITION_DIRECTION = -1;
    static constexpr float MIN_ANGLE = -180.0f;
    static constexpr float MAX_ANGLE = 180.0f;
}

namespace JAWS
{
    namespace DRIVER_A
    {                                                                       // assuming left
        static constexpr pin_pair_t DRIVER_PINS{GPIO_NUM_47, GPIO_NUM_21};  // driver 5
        static constexpr EncoderId ENCODER_ID = EncoderId::JawsLeft;        // for reading from bus
        static constexpr uint8_t ENCODER_BUS = 1;                           // RS485 bus, 1 or 2
        static constexpr uint8_t ENCODER_SLAVE = 3;                         // Modbus address = DIP value + 1
        static constexpr uint8_t GROUP_ID = 0x07;                           // for sending over canbus
    }
    namespace DRIVER_B
    {                                                                       // assuming Right
        static constexpr pin_pair_t DRIVER_PINS{GPIO_NUM_14, GPIO_NUM_13};  // driver 6
        static constexpr EncoderId ENCODER_ID = EncoderId::JawsRight;       // for reading from bus
        static constexpr uint8_t ENCODER_BUS = 2;                           // RS485 bus, 1 or 2
        static constexpr uint8_t ENCODER_SLAVE = 3;                         // Modbus address = DIP value + 1
        static constexpr uint8_t GROUP_ID = 0x08;                           // for sending over canbus
    }
    static constexpr gpio_num_t CURRENT_SENSE = bsp::A5;
    static constexpr gpio_num_t FAULT = GPIO_NUM_6;
    static constexpr int8_t SPEED_DIRECTION = 1;
    static constexpr int8_t POSITION_DIRECTION = -1;
    static constexpr float MIN_ANGLE = 0.0f;   // clenched
    static constexpr float MAX_ANGLE = 36.0f;  // safe opening
    static constexpr float HOME_BITE_CURRENT = 1.5f;
    static constexpr float HOME_IDLE_CURRENT = 0.2f;
}