#include <Arduino.h>

#include <hi_can_twai.hpp>
#include <optional>

#include "encoder_bus.hpp"
#include "esp_log.h"
#include "esp_system.h"
#include "excavation_config.hpp"
#include "motor_bank.hpp"
#include "motor_bank_parameter_group.hpp"

static constexpr gpio_num_t NSLEEP = GPIO_NUM_40;

using namespace hi_can;
using namespace hi_can::addressing;

static const char* const TAG = "main";  // For ESP_LOGing

TwaiInterface* can_interface = nullptr;
std::optional<PacketManager> packet_manager;

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

    // Built here rather than as globals: their constructors set up pins and
    // FreeRTOS mutexes, which isn't safe before setup() runs.
    static MotorBank lift(LIFT);
    static MotorBank tilt(TILT);
    static MotorBank jaws(JAWS);

    auto& interface = TwaiInterface::get_instance(
        std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
        filter_t{
            .address = static_cast<flagged_address_t>(kBucketAddress),
            .mask = DEVICE_MASK,
        });
    can_interface = &interface;
    packet_manager.emplace(interface);

    packet_manager->add_group(MotorBankParameterGroup(LIFT, lift));
    packet_manager->add_group(MotorBankParameterGroup(TILT, tilt));
    packet_manager->add_group(MotorBankParameterGroup(JAWS, jaws));

    // Failure is logged inside begin();
    encoder_bus().begin();

    // Drives the motors for SET_SPEED and SET_POSITION alike, so every bank needs it.
    MotorBank::start_control_task({&lift, &tilt, &jaws});
}

void loop()  // Add timeout for receiving velocity
{
    // Bus-off recovery first, so a restarted driver is running before the transmit pass.
    can_interface->handle();  // This is the ESP's specific handler with twai, error handling and recovery
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
