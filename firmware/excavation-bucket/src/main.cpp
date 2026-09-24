#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/sdm.h>

#include <chrono>
#include <hi_can_twai.hpp>
#include <optional>
#include <thread>

#include "bank_control_task.hpp"
#include "bank_encoder_map.hpp"
#include "encoder_bus.hpp"
#include "encoder_parameter_group.hpp"
#include "excavation_config.hpp"
#include "hi_can_address.hpp"
#include "motor_bank.hpp"
#include "motor_bank_parameter_group.hpp"
#include "motor_driver.hpp"
#include "motor_parameter_group.hpp"
#include "shared_memory.hpp"

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
std::optional<MotorParameterGroup> motor_lift_l_parameter_group;
std::optional<MotorParameterGroup> motor_lift_r_parameter_group;
std::optional<MotorParameterGroup> motor_tilt_l_parameter_group;
std::optional<MotorParameterGroup> motor_tilt_r_parameter_group;
std::optional<MotorParameterGroup> motor_jaws_l_parameter_group;
std::optional<MotorParameterGroup> motor_jaws_r_parameter_group;

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

    EncoderBus& encoderBusInstance = encoder_bus();

    motor_bank_lift.emplace(LIFT::DRIVER_A::DRIVER_PINS, LIFT::DRIVER_A::ENCODER_ID, LIFT::DRIVER_A::GROUP_ID,
                            LIFT::DRIVER_B::DRIVER_PINS, LIFT::DRIVER_B::ENCODER_ID, LIFT::DRIVER_B::GROUP_ID,
                            LIFT::CURRENT_SENSE, LIFT::FAULT, &encoderBusInstance);
    motor_bank_jaws.emplace(JAWS::DRIVER_A::DRIVER_PINS, JAWS::DRIVER_A::ENCODER_ID, JAWS::DRIVER_A::GROUP_ID,
                            JAWS::DRIVER_B::DRIVER_PINS, JAWS::DRIVER_B::ENCODER_ID, JAWS::DRIVER_B::GROUP_ID,
                            JAWS::CURRENT_SENSE, JAWS::FAULT, &encoderBusInstance);
    motor_bank_tilt.emplace(TILT::DRIVER_A::DRIVER_PINS, TILT::DRIVER_A::ENCODER_ID, TILT::DRIVER_A::GROUP_ID,
                            TILT::DRIVER_B::DRIVER_PINS, TILT::DRIVER_B::ENCODER_ID, TILT::DRIVER_B::GROUP_ID,
                            TILT::CURRENT_SENSE, TILT::FAULT, &encoderBusInstance);

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

    motor_lift_l_parameter_group.emplace(bucket::controller::encoder_group::LIFT_L, motor_bank_lift->get_driver_A());
    motor_lift_r_parameter_group.emplace(bucket::controller::encoder_group::LIFT_R, motor_bank_lift->get_driver_B());
    motor_tilt_l_parameter_group.emplace(bucket::controller::encoder_group::TILT_L, motor_bank_tilt->get_driver_A());
    motor_tilt_r_parameter_group.emplace(bucket::controller::encoder_group::TILT_R, motor_bank_tilt->get_driver_B());
    motor_jaws_l_parameter_group.emplace(bucket::controller::encoder_group::JAWS_L, motor_bank_jaws->get_driver_A());
    motor_jaws_r_parameter_group.emplace(bucket::controller::encoder_group::JAWS_R, motor_bank_jaws->get_driver_B());

    packet_manager->add_group(motor_lift_l_parameter_group.value());
    packet_manager->add_group(motor_lift_r_parameter_group.value());
    packet_manager->add_group(motor_tilt_l_parameter_group.value());
    packet_manager->add_group(motor_tilt_r_parameter_group.value());
    packet_manager->add_group(motor_jaws_l_parameter_group.value());
    packet_manager->add_group(motor_jaws_r_parameter_group.value());

    // Failure is logged inside begin();
    encoderBusInstance.begin();
}

void loop()
{
    // TODO: will this be fast enough?
    motor_bank_lift->monitor_and_move();
    motor_bank_jaws->monitor_and_move();
    motor_bank_tilt->monitor_and_move();
    packet_manager->handle();
    delay(1);
}
