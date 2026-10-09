#pragma once

#include <cstddef>
#include <cstdint>
#include <iterator>

#include "board_support.hpp"
#include "encoder_bus.hpp"
#include "hi_can_address.hpp"

namespace bucket_can = hi_can::addressing::excavation::bucket::controller;

// One actuator: its motor driver and the encoder board it reads.
struct DriverConfig
{
    bsp::pin_pair_t pins;
    EncoderId encoder;                    // for reading from bus
    uint8_t encoder_bus;                  // RS485 bus, 1 or 2
    uint8_t encoder_slave;                // Modbus address = DIP value + 1
    bucket_can::encoder_group can_group;  // for sending GET_ANGLE over canbus
};

// A pair of actuators moving one joint.
struct BankConfig
{
    const char* name;
    bucket_can::bank_group can_group;  // for commands and reports over canbus
    DriverConfig a;                    // assuming left
    DriverConfig b;                    // assuming right
    gpio_num_t current_sense;
    gpio_num_t fault;
    int8_t speed_direction;
    int8_t position_direction;
    float min_angle;
    float max_angle;
    float home_bite_current = 0.0f;  // 0 = this bank doesn't home
    float home_idle_current = 0.0f;
};

// Note:
// We're only using the analog functions on the current sense pins,
// so these are the only ones named with analog numbers
// even though all the pins can do analog and digital IO

// Encoders: each driver's encoder_bus and encoder_slave say which encoder board it
// reads. encoder_slave is the Modbus address, i.e. the board's DIP value + 1.
// Left encoders are on bus 1 and right on bus 2, so one cable fault leaves each
// joint with a working encoder. Checked at compile time in encoder_bus.cpp.

// Directions: 1 or -1 per bank, and independent of each other.
// speed_direction: flip if a positive SET_SPEED moves the joint the wrong way.
// position_direction: flip if SET_POSITION drives away from the target, i.e.
// use -1 when driving the motors forward makes the encoder angle go down.

// Limits: a bank won't drive below min_angle or above max_angle (degrees). Lift
// and tilt have none yet, so theirs are the full range.

// Homing (jaws only, started by SET_ZERO_POS): opens until the current falls
// below home_idle_current, then bites until it passes home_bite_current and
// backs off, a few times. The encoders are zeroed during the last bite, so 0
// degrees is the clenched frame. Amps, as GET_CURRENT reports them. Opening
// must raise the angle. Timing is in motor_bank.hpp (kHome...).

// TODO update a and b to be left and right based (once pins are confirmed)
inline constexpr BankConfig LIFT{
    .name = "lift",
    .can_group = bucket_can::bank_group::LIFT,
    .a = {
        .pins = {GPIO_NUM_16, GPIO_NUM_15},  // driver 1
        .encoder = EncoderId::LiftLeft,
        .encoder_bus = 1,
        .encoder_slave = 1,
        .can_group = bucket_can::encoder_group::LIFT_L,
    },
    .b = {
        .pins = {GPIO_NUM_41, GPIO_NUM_42},  // driver 2
        .encoder = EncoderId::LiftRight,
        .encoder_bus = 2,
        .encoder_slave = 1,
        .can_group = bucket_can::encoder_group::LIFT_R,
    },
    .current_sense = bsp::A1,
    .fault = GPIO_NUM_2,
    .speed_direction = 1,
    .position_direction = -1,
    .min_angle = -180.0f,
    .max_angle = 180.0f,
};

inline constexpr BankConfig TILT{
    .name = "tilt",
    .can_group = bucket_can::bank_group::TILT,
    .a = {
        .pins = {GPIO_NUM_38, GPIO_NUM_37},  // driver 3
        .encoder = EncoderId::TiltLeft,
        .encoder_bus = 1,
        .encoder_slave = 2,
        .can_group = bucket_can::encoder_group::TILT_L,
    },
    .b = {
        .pins = {GPIO_NUM_45, GPIO_NUM_48},  // driver 4
        .encoder = EncoderId::TiltRight,
        .encoder_bus = 2,
        .encoder_slave = 2,
        .can_group = bucket_can::encoder_group::TILT_R,
    },
    .current_sense = bsp::A3,
    .fault = GPIO_NUM_4,
    .speed_direction = 1,
    .position_direction = -1,
    .min_angle = -180.0f,
    .max_angle = 180.0f,
};

inline constexpr BankConfig JAWS{
    .name = "jaws",
    .can_group = bucket_can::bank_group::JAWS,
    .a = {
        .pins = {GPIO_NUM_47, GPIO_NUM_21},  // driver 5
        .encoder = EncoderId::JawsLeft,
        .encoder_bus = 1,
        .encoder_slave = 3,
        .can_group = bucket_can::encoder_group::JAWS_L,
    },
    .b = {
        .pins = {GPIO_NUM_14, GPIO_NUM_13},  // driver 6
        .encoder = EncoderId::JawsRight,
        .encoder_bus = 2,
        .encoder_slave = 3,
        .can_group = bucket_can::encoder_group::JAWS_R,
    },
    .current_sense = bsp::A5,
    .fault = GPIO_NUM_6,
    .speed_direction = 1,
    .position_direction = -1,
    .min_angle = 0.0f,   // clenched
    .max_angle = 36.0f,  // safe opening
    .home_bite_current = 1.5f,
    .home_idle_current = 0.2f,
};

inline constexpr const BankConfig* kBanks[] = {&LIFT, &TILT, &JAWS};
inline constexpr size_t kBankCount = std::size(kBanks);
