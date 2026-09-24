#pragma once

#include "board_support.hpp"
#include "encoder_bus.hpp"

using namespace bsp;

// Note:
// We're only using the analog functions on the current sense pins,
// so these are the only ones named with analog numbers
// even though all the pins can do analog and digital IO

namespace LIFT
{
    // TODO update namespace to be LEFT and RIGHT based (once pins are confirmed)
    namespace DRIVER_A
    {                                                                       // assuming left
        static constexpr pin_pair_t DRIVER_PINS{GPIO_NUM_15, GPIO_NUM_16};  // driver 1
        static constexpr EncoderId ENCODER_ID = EncoderId::LiftLeft;        // for reading from bus
        static constexpr uint8_t GROUP_ID = 0x03;                           // for sending over canbus
    }
    namespace DRIVER_B
    {                                                                       // assuming Right
        static constexpr pin_pair_t DRIVER_PINS{GPIO_NUM_42, GPIO_NUM_41};  // driver 2
        static constexpr EncoderId ENCODER_ID = EncoderId::LiftRight;       // for reading from bus
        static constexpr uint8_t GROUP_ID = 0x04;                           // for sending over canbus
    }
    static constexpr gpio_num_t CURRENT_SENSE = bsp::A1;
    static constexpr gpio_num_t FAULT = GPIO_NUM_2;
}

namespace TILT
{
    namespace DRIVER_A
    {                                                                       // assuming left
        static constexpr pin_pair_t DRIVER_PINS{GPIO_NUM_47, GPIO_NUM_21};  // driver 5
        static constexpr EncoderId ENCODER_ID = EncoderId::TiltLeft;        // for reading from bus
        static constexpr uint8_t GROUP_ID = 0x05;                           // for sending over canbus
    }
    namespace DRIVER_B
    {                                                                       // assuming Right
        static constexpr pin_pair_t DRIVER_PINS{GPIO_NUM_14, GPIO_NUM_13};  // driver 6
        static constexpr EncoderId ENCODER_ID = EncoderId::TiltRight;       // for reading from bus
        static constexpr uint8_t GROUP_ID = 0x06;                           // for sending over canbus
    }
    static constexpr gpio_num_t CURRENT_SENSE = bsp::A5;
    static constexpr gpio_num_t FAULT = GPIO_NUM_6;
}

namespace JAWS
{
    namespace DRIVER_A
    {                                                                       // assuming left
        static constexpr pin_pair_t DRIVER_PINS{GPIO_NUM_38, GPIO_NUM_37};  // driver 3
        static constexpr EncoderId ENCODER_ID = EncoderId::JawsLeft;        // for reading from bus
        static constexpr uint8_t GROUP_ID = 0x07;                           // for sending over canbus
    }
    namespace DRIVER_B
    {                                                                       // assuming Right
        static constexpr pin_pair_t DRIVER_PINS{GPIO_NUM_45, GPIO_NUM_48};  // driver 4
        static constexpr EncoderId ENCODER_ID = EncoderId::JawsRight;       // for reading from bus
        static constexpr uint8_t GROUP_ID = 0x08;                           // for sending over canbus
    }
    static constexpr gpio_num_t CURRENT_SENSE = bsp::A3;
    static constexpr gpio_num_t FAULT = GPIO_NUM_4;
}