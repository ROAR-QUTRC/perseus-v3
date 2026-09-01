#include <Arduino.h>
#include <driver/sdm.h>

#include <board_support.hpp>
#include <chrono>
#include <hi_can_twai.hpp>
#include <optional>
#include <thread>

#include "motor_bank.hpp"
#include "motor_driver.hpp"

using namespace bsp;

static constexpr pin_pair_t DRIVER_1_PINS{GPIO_NUM_15, GPIO_NUM_16};
static constexpr pin_pair_t DRIVER_2_PINS{GPIO_NUM_42, GPIO_NUM_41};
static constexpr pin_pair_t DRIVER_3_PINS{GPIO_NUM_38, GPIO_NUM_37};
static constexpr pin_pair_t DRIVER_4_PINS{GPIO_NUM_45, GPIO_NUM_48};
static constexpr pin_pair_t DRIVER_5_PINS{GPIO_NUM_47, GPIO_NUM_21};
static constexpr pin_pair_t DRIVER_6_PINS{GPIO_NUM_14, GPIO_NUM_13};

static constexpr gpio_num_t BANK_1_CURRENT_LIMIT = GPIO_NUM_40;
static constexpr gpio_num_t BANK_2_CURRENT_LIMIT = bsp::A10;
static constexpr gpio_num_t BANK_3_CURRENT_LIMIT = GPIO_NUM_12;

static constexpr gpio_num_t BANK_1_CURRENT_SENSE = bsp::A1;
static constexpr gpio_num_t BANK_2_CURRENT_SENSE = bsp::A3;
static constexpr gpio_num_t BANK_3_CURRENT_SENSE = bsp::A5;

static constexpr gpio_num_t BANK_1_FAULT = bsp::A2;
static constexpr gpio_num_t BANK_2_FAULT = bsp::A4;
static constexpr gpio_num_t BANK_3_FAULT = bsp::A6;

static constexpr gpio_num_t SLEEP = GPIO_NUM_39;

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace hi_can;
using namespace hi_can::addressing;

std::optional<PacketManager> packet_manager;

void handle_motor_speed_data(const Packet& packet);
void handle_motor_current_data(const Packet& packet);
void set_motor_speed(const excavation::bucket::controller::group& group,
                     const int16_t& speed);
void set_motor_current(const excavation::bucket::controller::group& group,
                       const uint16_t& current);
void register_motor_bank(const excavation::bucket::controller::group& group,
                         const uint8_t& speed_param);

std::optional<MotorBank> motor_bank_1;
std::optional<MotorBank> motor_bank_2;
std::optional<MotorBank> motor_bank_3;

constexpr standard_address_t DEVICE_ADDRESS{
    excavation::SYSTEM_ID,
    excavation::bucket::SUBSYSTEM_ID,
    excavation::bucket::controller::DEVICE_ID,
};

void setup()
{
    // reset drivers
    pinMode(SLEEP, OUTPUT);
    digitalWrite(SLEEP, LOW);
    delay(100);
    digitalWrite(SLEEP, HIGH);
    motor_bank_1.emplace(DRIVER_1_PINS, DRIVER_2_PINS, BANK_1_CURRENT_LIMIT,
                         BANK_1_CURRENT_SENSE, BANK_1_FAULT);
    motor_bank_2.emplace(DRIVER_3_PINS, DRIVER_4_PINS, BANK_2_CURRENT_LIMIT,
                         BANK_2_CURRENT_SENSE, BANK_2_FAULT);
    motor_bank_3.emplace(DRIVER_5_PINS, DRIVER_6_PINS, BANK_3_CURRENT_LIMIT,
                         BANK_3_CURRENT_SENSE, BANK_3_FAULT);

    auto& interface = TwaiInterface::get_instance(
        std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
        filter_t{
            .address = static_cast<flagged_address_t>(DEVICE_ADDRESS),
            .mask = DEVICE_MASK,
        });
    packet_manager.emplace(interface);

    using namespace excavation::bucket::controller;
    const std::vector<group> actuator_groups = {
        group::LIFT_BOTH,
        group::LIFT_LEFT,
        group::LIFT_RIGHT,
        group::TILT_BOTH,
        group::TILT_LEFT,
        group::TILT_RIGHT,
        group::JAWS_BOTH,
        group::JAWS_LEFT,
        group::JAWS_RIGHT,
    };

    for (const auto& group : actuator_groups)
        register_motor_bank(group, static_cast<uint8_t>(actuator_parameter::SPEED));

    using namespace parameters::excavation::bucket::controller;
    packet_manager->set_transmission_config(
        static_cast<flagged_address_t>(standard_address_t{
            DEVICE_ADDRESS, static_cast<uint8_t>(group::BANK_1),
            static_cast<uint8_t>(bank_parameter::STATUS)}),
        {
            .generator =
                [=]()
            {
                return current_t{
                    static_cast<uint16_t>(
                        std::clamp(motor_bank_1->get_average_current() * 1000,
                                   static_cast<float>(
                                       std::numeric_limits<uint16_t>::min()),
                                   static_cast<float>(
                                       std::numeric_limits<uint16_t>::max())))}
                    .serialize_data();
            },
            .interval = 100ms,
            .should_transmit_immediately = true,
        });
    packet_manager->set_callback(
        filter_t{static_cast<flagged_address_t>(standard_address_t{
            DEVICE_ADDRESS, static_cast<uint8_t>(group::BANK_1),
            static_cast<uint8_t>(bank_parameter::CURRENT_LIMIT)})},
        {
            .data_callback = handle_motor_current_data,
        });
    packet_manager->set_transmission_config(
        static_cast<flagged_address_t>(standard_address_t{
            DEVICE_ADDRESS, static_cast<uint8_t>(group::BANK_2),
            static_cast<uint8_t>(bank_parameter::STATUS)}),
        {
            .generator =
                [=]()
            {
                return current_t{
                    static_cast<uint16_t>(
                        std::clamp(motor_bank_2->get_average_current() * 1000,
                                   static_cast<float>(
                                       std::numeric_limits<uint16_t>::min()),
                                   static_cast<float>(
                                       std::numeric_limits<uint16_t>::max())))}
                    .serialize_data();
            },
            .interval = 100ms,
            .should_transmit_immediately = true,
        });
    packet_manager->set_callback(
        filter_t{static_cast<flagged_address_t>(standard_address_t{
            DEVICE_ADDRESS, static_cast<uint8_t>(group::BANK_2),
            static_cast<uint8_t>(bank_parameter::CURRENT_LIMIT)})},
        {
            .data_callback = handle_motor_current_data,
        });
    packet_manager->set_transmission_config(
        static_cast<flagged_address_t>(standard_address_t{
            DEVICE_ADDRESS, static_cast<uint8_t>(group::BANK_3),
            static_cast<uint8_t>(bank_parameter::STATUS)}),
        {
            .generator =
                [=]()
            {
                return current_t{
                    static_cast<uint16_t>(
                        std::clamp(motor_bank_3->get_average_current() * 1000,
                                   static_cast<float>(
                                       std::numeric_limits<uint16_t>::min()),
                                   static_cast<float>(
                                       std::numeric_limits<uint16_t>::max())))}
                    .serialize_data();
            },
            .interval = 100ms,
            .should_transmit_immediately = true,
        });
    packet_manager->set_callback(
        filter_t{static_cast<flagged_address_t>(standard_address_t{
            DEVICE_ADDRESS, static_cast<uint8_t>(group::BANK_3),
            static_cast<uint8_t>(bank_parameter::CURRENT_LIMIT)})},
        {
            .data_callback = handle_motor_current_data,
        });
}

void loop()
{
    packet_manager->handle();
    delay(1);
}

void handle_motor_speed_data(const Packet& packet)
{
    using namespace excavation::bucket::controller;
    using namespace hi_can::parameters::excavation::bucket::controller;
    try
    {
        standard_address_t address{packet.get_address().address};
        set_motor_speed(static_cast<group>(
                            standard_address_t(packet.get_address().address).group),
                        speed_t{packet.get_data()}.value);
    }
    catch (const std::exception& e)
    {
        printf(std::format("Failed to parse speed packet: {}\n", e.what()).c_str());
    }
}
void handle_motor_current_data(const Packet& packet)
{
    using namespace excavation::bucket::controller;
    using namespace hi_can::parameters::excavation::bucket::controller;
    try
    {
        standard_address_t address{packet.get_address().address};
        set_motor_current(
            static_cast<group>(
                standard_address_t(packet.get_address().address).group),
            current_t{packet.get_data()}.value);
    }
    catch (const std::exception& e)
    {
        printf(
            std::format("Failed to parse current packet: {}\n", e.what()).c_str());
    }
}

void set_motor_speed(const excavation::bucket::controller::group& group,
                     const int16_t& speed)
{
    using namespace excavation::bucket::controller;
    // printf(std::format("Setting motor (group) {:#x} to speed {}\n",
    //                    static_cast<uint8_t>(group), speed)
    //            .c_str());
    switch (group)
    {
    case group::LIFT_BOTH:
        set_motor_speed(group::LIFT_LEFT, speed);
        set_motor_speed(group::LIFT_RIGHT, speed);
        break;
    case group::LIFT_LEFT:
        motor_bank_1->set_speed_a(speed);
        break;
    case group::LIFT_RIGHT:
        motor_bank_1->set_speed_b(speed);
        break;
    case group::TILT_BOTH:
        motor_bank_3->set_speed_a(speed);
        break;
    case group::JAWS_BOTH:
        set_motor_speed(group::JAWS_LEFT, speed);
        set_motor_speed(group::JAWS_RIGHT, speed);
        break;
    case group::JAWS_LEFT:
        motor_bank_2->set_speed_a(speed);
        break;
    case group::JAWS_RIGHT:
        motor_bank_2->set_speed_b(speed);
        break;
    default:
        break;
    }
}
void set_motor_current(const excavation::bucket::controller::group& group,
                       const uint16_t& current)
{
    using namespace excavation::bucket::controller;
    printf(std::format("Setting {:#x} current to {}mA\n",
                       static_cast<uint8_t>(group), current)
               .c_str());
    switch (group)
    {
    case group::BANK_1:
        motor_bank_1->set_current_limit(current / 1000.0f);
        break;
    case group::BANK_2:
        motor_bank_2->set_current_limit(current / 1000.0f);
        break;
    case group::BANK_3:
        motor_bank_3->set_current_limit(current / 1000.0f);
        break;
    default:
        break;
    }
}

void register_motor_bank(const excavation::bucket::controller::group& group,
                         const uint8_t& speed_param)
{
    const standard_address_t speed_address{DEVICE_ADDRESS,
                                           static_cast<uint8_t>(group),
                                           static_cast<uint8_t>(speed_param)};
    packet_manager->set_callback(
        filter_t{
            static_cast<flagged_address_t>(speed_address),
        },
        {
            .data_callback = handle_motor_speed_data,
            .timeout_callback = std::bind(set_motor_speed, group, (int16_t)0),
            .timeout = 200ms,
        });
}
