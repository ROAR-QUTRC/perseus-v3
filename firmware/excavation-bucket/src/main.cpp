#include <Arduino.h>
#include <driver/sdm.h>

#include <board_support.hpp>
#include <chrono>
#include <hi_can_twai.hpp>
#include <optional>
#include <thread>

#include "encoder_bus.hpp"
#include "encoder_parameter_group.hpp"
#include "hi_can_address.hpp"
#include "motor_bank.hpp"
#include "motor_bank_parameter_group.hpp"
#include "motor_driver.hpp"

using namespace bsp;

static constexpr pin_pair_t DRIVER_1_PINS{GPIO_NUM_15, GPIO_NUM_16};
static constexpr pin_pair_t DRIVER_2_PINS{GPIO_NUM_42, GPIO_NUM_41};
static constexpr pin_pair_t DRIVER_3_PINS{GPIO_NUM_38, GPIO_NUM_37};
static constexpr pin_pair_t DRIVER_4_PINS{GPIO_NUM_45, GPIO_NUM_48};
static constexpr pin_pair_t DRIVER_5_PINS{GPIO_NUM_47, GPIO_NUM_21};
static constexpr pin_pair_t DRIVER_6_PINS{GPIO_NUM_14, GPIO_NUM_13};

// We're only using the analog functions on the current sense pins,
// so these are the only ones named with analog numbers
// even though all the pins can do analog and digital IO
static constexpr gpio_num_t BANK_1_CURRENT_SENSE = bsp::A1;
static constexpr gpio_num_t BANK_2_CURRENT_SENSE = bsp::A3;
static constexpr gpio_num_t BANK_3_CURRENT_SENSE = bsp::A5;

static constexpr gpio_num_t BANK_1_FAULT = GPIO_NUM_2;
static constexpr gpio_num_t BANK_2_FAULT = GPIO_NUM_4;
static constexpr gpio_num_t BANK_3_FAULT = GPIO_NUM_6;

static constexpr gpio_num_t NSLEEP = GPIO_NUM_40;

static constexpr gpio_num_t MCU1_DIR = GPIO_NUM_7;
static constexpr gpio_num_t MCU2_DIR = GPIO_NUM_39;

static constexpr gpio_num_t MCU1_RX = GPIO_NUM_8;
static constexpr gpio_num_t MCU1_TX = GPIO_NUM_9;
static constexpr gpio_num_t MCU2_RX = GPIO_NUM_10;
static constexpr gpio_num_t MCU2_TX = GPIO_NUM_12;

using namespace hi_can;
using namespace hi_can::addressing;
using namespace hi_can::addressing::excavation;

std::optional<PacketManager> packet_manager;

std::optional<MotorBank> motor_bank_lift;  // Bank 1
std::optional<MotorBank> motor_bank_jaws;  // Bank 2
std::optional<MotorBank> motor_bank_tilt;  // Bank 3
std::optional<MotorBankParameterGroup> motor_bank_lift_parameter_group;
std::optional<MotorBankParameterGroup> motor_bank_jaws_parameter_group;
std::optional<MotorBankParameterGroup> motor_bank_tilt_parameter_group;

std::optional<EncoderParameterGroup> encoder_lift_1_group;  // LiftLeft
std::optional<EncoderParameterGroup> encoder_lift_2_group;  // LiftRight
std::optional<EncoderParameterGroup> encoder_jaws_1_group;  // JawsLeft
std::optional<EncoderParameterGroup> encoder_jaws_2_group;  // JawsRight
std::optional<EncoderParameterGroup> encoder_tilt_1_group;  // TiltLeft
std::optional<EncoderParameterGroup> encoder_tilt_2_group;  // TiltRight

constexpr standard_address_t DEVICE_ADDRESS{
    SYSTEM_ID,
    bucket::SUBSYSTEM_ID,
    bucket::controller::DEVICE_ID,
};

void setup()
{
    // reset drivers
    pinMode(NSLEEP, OUTPUT);
    digitalWrite(NSLEEP, LOW);
    delay(100);
    digitalWrite(NSLEEP, HIGH);

    motor_bank_lift.emplace(DRIVER_1_PINS, DRIVER_2_PINS,
                            BANK_1_CURRENT_SENSE, BANK_1_FAULT);
    motor_bank_jaws.emplace(DRIVER_3_PINS, DRIVER_4_PINS,
                            BANK_2_CURRENT_SENSE, BANK_2_FAULT);
    motor_bank_tilt.emplace(DRIVER_5_PINS, DRIVER_6_PINS,
                            BANK_3_CURRENT_SENSE, BANK_3_FAULT);

    motor_bank_lift_parameter_group.emplace(bucket::controller::bank_group::LIFT, motor_bank_lift.value());
    motor_bank_jaws_parameter_group.emplace(bucket::controller::bank_group::JAWS, motor_bank_jaws.value());
    motor_bank_tilt_parameter_group.emplace(bucket::controller::bank_group::TILT, motor_bank_tilt.value());

    auto& interface = TwaiInterface::get_instance(
        std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
        filter_t{
            .address = static_cast<flagged_address_t>(DEVICE_ADDRESS),
            .mask = DEVICE_MASK,
        });
    packet_manager.emplace(interface);

    // Add the CAN motor groups
    packet_manager->add_group(motor_bank_lift_parameter_group.value());
    packet_manager->add_group(motor_bank_jaws_parameter_group.value());
    packet_manager->add_group(motor_bank_tilt_parameter_group.value());

    // Failure is logged inside begin(); the motor banks work without encoder data.
    encoder_bus().begin();

    // Each replies to a GET_ANGLE request (RTR) with that encoder's latest
    // cached reading - nothing is sent until asked.
    encoder_lift_1_group.emplace(EncoderId::LiftLeft, bucket::controller::encoder_group::LIFT_L, interface);
    encoder_lift_2_group.emplace(EncoderId::LiftRight, bucket::controller::encoder_group::LIFT_R, interface);
    encoder_jaws_1_group.emplace(EncoderId::JawsLeft, bucket::controller::encoder_group::JAWS_L, interface);
    encoder_jaws_2_group.emplace(EncoderId::JawsRight, bucket::controller::encoder_group::JAWS_R, interface);
    encoder_tilt_1_group.emplace(EncoderId::TiltLeft, bucket::controller::encoder_group::TILT_L, interface);
    encoder_tilt_2_group.emplace(EncoderId::TiltRight, bucket::controller::encoder_group::TILT_R, interface);

    // Add the CAN encoder groups
    packet_manager->add_group(encoder_lift_1_group.value());
    packet_manager->add_group(encoder_lift_2_group.value());

    packet_manager->add_group(encoder_jaws_1_group.value());
    packet_manager->add_group(encoder_jaws_2_group.value());

    packet_manager->add_group(encoder_tilt_1_group.value());
    packet_manager->add_group(encoder_tilt_2_group.value());
}

void loop()
{
    packet_manager->handle();
    delay(1);  // change to consistent time step with vTaskDelayUntil as PID or closed loop feedback would jitter, at the very least time drift
}
