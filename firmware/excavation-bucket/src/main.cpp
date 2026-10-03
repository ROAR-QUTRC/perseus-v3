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
#include "esp_log.h"
#include "esp_system.h"
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

static const char* const TAG = "main";

TwaiInterface* can_interface = nullptr;
std::optional<PacketManager> packet_manager;

/* This order is incorrect, it is lift, tilt and jaws.*/
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

// Tells a crash (panic) apart from a watchdog reset, a brownout or a power cycle.
static const char* reset_reason_name(esp_reset_reason_t reason)
{
    switch (reason)
    {
    case ESP_RST_POWERON:
        return "power-on";
    case ESP_RST_EXT:
        return "reset pin";
    case ESP_RST_SW:
        return "software restart";
    case ESP_RST_PANIC:
        return "panic (crash)";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
        return "watchdog";
    case ESP_RST_BROWNOUT:
        return "brownout";
    default:
        return "other";
    }
}

void setup()
{
    const esp_reset_reason_t reset_reason = esp_reset_reason();
    ESP_LOGW(TAG, "reset reason: %s (%d)", reset_reason_name(reset_reason), static_cast<int>(reset_reason));

    // reset drivers
    pinMode(NSLEEP, OUTPUT);
    digitalWrite(NSLEEP, LOW);
    delay(100);
    digitalWrite(NSLEEP, HIGH);

    EncoderBus& encoderBusInstance = encoder_bus();

    motor_bank_lift.emplace(LIFT::DRIVER_A::DRIVER_PINS, LIFT::DRIVER_A::ENCODER_ID, LIFT::DRIVER_A::GROUP_ID,
                            LIFT::DRIVER_B::DRIVER_PINS, LIFT::DRIVER_B::ENCODER_ID, LIFT::DRIVER_B::GROUP_ID,
                            LIFT::CURRENT_SENSE, LIFT::FAULT, LIFT::SPEED_DIRECTION, LIFT::POSITION_DIRECTION,
                            LIFT::MIN_ANGLE, LIFT::MAX_ANGLE, &encoderBusInstance);
    motor_bank_jaws.emplace(JAWS::DRIVER_A::DRIVER_PINS, JAWS::DRIVER_A::ENCODER_ID, JAWS::DRIVER_A::GROUP_ID,
                            JAWS::DRIVER_B::DRIVER_PINS, JAWS::DRIVER_B::ENCODER_ID, JAWS::DRIVER_B::GROUP_ID,
                            JAWS::CURRENT_SENSE, JAWS::FAULT, JAWS::SPEED_DIRECTION, JAWS::POSITION_DIRECTION,
                            JAWS::MIN_ANGLE, JAWS::MAX_ANGLE, &encoderBusInstance);
    motor_bank_tilt.emplace(TILT::DRIVER_A::DRIVER_PINS, TILT::DRIVER_A::ENCODER_ID, TILT::DRIVER_A::GROUP_ID,
                            TILT::DRIVER_B::DRIVER_PINS, TILT::DRIVER_B::ENCODER_ID, TILT::DRIVER_B::GROUP_ID,
                            TILT::CURRENT_SENSE, TILT::FAULT, TILT::SPEED_DIRECTION, TILT::POSITION_DIRECTION,
                            TILT::MIN_ANGLE, TILT::MAX_ANGLE, &encoderBusInstance);

    motor_bank_jaws->enable_homing(JAWS::HOME_BITE_CURRENT, JAWS::HOME_IDLE_CURRENT);

    motor_bank_lift_parameter_group.emplace(bucket::controller::bank_group::LIFT, motor_bank_lift.value());
    motor_bank_jaws_parameter_group.emplace(bucket::controller::bank_group::JAWS, motor_bank_jaws.value());
    motor_bank_tilt_parameter_group.emplace(bucket::controller::bank_group::TILT, motor_bank_tilt.value());

    auto& interface = TwaiInterface::get_instance(
        std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
        filter_t{
            .address = static_cast<flagged_address_t>(DEVICE_ADDRESS),
            .mask = DEVICE_MASK,
        });
    can_interface = &interface;
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

    // Drives the motors for SET_SPEED and SET_POSITION alike, so every bank needs it.
    start_bank_control_task({&motor_bank_lift.value(), &motor_bank_jaws.value(), &motor_bank_tilt.value()});
}

void loop()  // Add timeout for receiving velocity
{
    motor_bank_lift->monitor_and_move();
    motor_bank_jaws->monitor_and_move();
    motor_bank_tilt->monitor_and_move();

    // Bus-off recovery first, so a restarted driver is running before the transmit pass.
    can_interface->handle();
    try
    {
        packet_manager->handle();
    }
    catch (const std::exception& e)
    {
        // Bus faults no longer throw (transmit drops the frame). This is for
        // malformed frames or hi-can bugs: staying on the bus beats a reboot,
        // which costs ROS ~10 s of encoder rediscovery.
        static uint32_t errors = 0;
        static uint32_t last_log_ms = 0;
        if (++errors == 1 || millis() - last_log_ms >= 1000)
        {
            last_log_ms = millis();
            ESP_LOGE(TAG, "CAN error #%lu: %s", static_cast<unsigned long>(errors), e.what());
        }
    }
    vTaskDelay(pdMS_TO_TICKS(1));  // Techically delay(1) works since arduino core treats delay as the same thing, but semantics
}
