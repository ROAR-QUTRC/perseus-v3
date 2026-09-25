// can_sim.cpp
//
// Bench entry point for env:can_sim, built instead of src/main.cpp. Replaces
// the TWAI bus with an in-memory interface and runs the lift bank only. Sends
// no commands: the bank stays in Velocity mode at speed 0, so the control
// task holds both H-bridges at zero while both lift encoders are read and
// printed on the console UART, along with what ROS would receive over CAN.

#include <Arduino.h>

#include <cstdio>
#include <deque>
#include <optional>

#include "bank_control_task.hpp"
#include "encoder_bus.hpp"
#include "excavation_config.hpp"
#include "hi_can.hpp"
#include "hi_can_address.hpp"
#include "motor_bank.hpp"
#include "motor_bank_parameter_group.hpp"
#include "motor_parameter_group.hpp"

using namespace hi_can;
using namespace hi_can::addressing;
using namespace hi_can::addressing::excavation::bucket::controller;

namespace
{
    constexpr gpio_num_t NSLEEP = GPIO_NUM_40;

    constexpr uint32_t kPrintPeriodMs = 250;

    constexpr uint8_t kSimEncoders = encoder_bit(LIFT::DRIVER_A::ENCODER_ID) | encoder_bit(LIFT::DRIVER_B::ENCODER_ID);

    constexpr standard_address_t DEVICE_ADDRESS{
        excavation::SYSTEM_ID,
        excavation::bucket::SUBSYSTEM_ID,
        excavation::bucket::controller::DEVICE_ID,
    };

    class SimInterface : public FilteredCanInterface
    {
    public:
        void transmit(const Packet& packet) override
        {
            if (tx_.size() >= kMaxTx)
                tx_.pop_front();
            tx_.push_back(packet);
        }

        std::optional<Packet> receive(bool) override { return std::nullopt; }  // nothing is ever sent to the board

        // Latest frame sent to `address`, as ROS would last have seen it.
        std::optional<Packet> last_sent(const flagged_address_t& address) const
        {
            for (auto it = tx_.rbegin(); it != tx_.rend(); ++it)
                if (it->get_address() == address)
                    return *it;
            return std::nullopt;
        }

    private:
        static constexpr size_t kMaxTx = 32;
        std::deque<Packet> tx_;
    };

    SimInterface sim;
    std::optional<PacketManager> packet_manager;
    std::optional<MotorBank> lift;
    std::optional<MotorBankParameterGroup> lift_group;
    std::optional<MotorParameterGroup> lift_left_group;
    std::optional<MotorParameterGroup> lift_right_group;

    const char* link_name(modbus::DeviceState state)
    {
        switch (state)
        {
        case modbus::DeviceState::Unknown:
            return "unknown";
        case modbus::DeviceState::Ok:
            return "ok";
        case modbus::DeviceState::Degraded:
            return "degraded";
        case modbus::DeviceState::Lost:
            return "lost";
        }
        return "?";
    }

    const char* mode_name(MotorBank::ControlMode mode)
    {
        return mode == MotorBank::ControlMode::Position ? "POS" : "VEL";
    }

    // Value of the last report sent to `address`, in degrees.
    void print_reported(const char* label, const flagged_address_t& address)
    {
        const std::optional<Packet> frame = sim.last_sent(address);
        const std::optional<int16_t> units = frame ? frame->get_data<int16_t>() : std::nullopt;
        if (units)
            printf("  %s %6.1f", label, *units / kPositionUnitsPerDegree);
        else
            printf("  %s     --", label);
    }

    flagged_address_t get_angle_address(encoder_group group)
    {
        return static_cast<flagged_address_t>(standard_address_t{
            DEVICE_ADDRESS, static_cast<uint8_t>(group), static_cast<uint8_t>(encoder_parameter::GET_ANGLE)});
    }

    void print_encoder(const char* side, EncoderId id, uint32_t now)
    {
        EncoderReading reading;
        encoder_bus().get(id, &reading);
        const std::optional<float> angle = MotorBank::encoder_degrees(id, now);
        const char* alive = !reading.status_valid ? "?" : reading.master_alive ? "yes"
                                                                               : "no";

        printf("  %s ", side);
        if (angle)
            printf("%7.2f", *angle);
        else if (!reading.present)
            printf("not found");
        else if (!reading.angle_valid)
            printf("invalid");
        else
            printf("stale %lums", static_cast<unsigned long>(reading.angle_age_ms(now)));
        printf(" (%s %lu/%lu mag %d alive %s)", link_name(reading.link), static_cast<unsigned long>(reading.stats.ok),
               static_cast<unsigned long>(reading.stats.total), reading.magnet_detected, alive);
    }

    void print_status(uint32_t now)
    {
        const MotorBank::Status status = lift->get_status();

        printf("%7.2fs %s out %d/%d", now / 1000.0f, encoder_bus().ready() ? "" : "[discovery]", status.output_a,
               status.output_b);
        if (status.mode != MotorBank::ControlMode::Velocity || status.speed != 0)
            printf(" !! %s speed %d", mode_name(status.mode), status.speed);  // should never happen here
        print_encoder("L", LIFT::DRIVER_A::ENCODER_ID, now);
        print_encoder("R", LIFT::DRIVER_B::ENCODER_ID, now);

        printf("  | can");
        print_reported("L", get_angle_address(encoder_group::LIFT_L));
        print_reported("R", get_angle_address(encoder_group::LIFT_R));
        print_reported("avg", static_cast<flagged_address_t>(standard_address_t{
                                  DEVICE_ADDRESS, static_cast<uint8_t>(bank_group::LIFT),
                                  static_cast<uint8_t>(bank_parameter::GET_POSITION)}));
        printf("\n");
    }
}  // namespace

void setup()
{
    pinMode(NSLEEP, OUTPUT);
    digitalWrite(NSLEEP, LOW);
    delay(100);
    digitalWrite(NSLEEP, HIGH);

    EncoderBus& bus = encoder_bus();
    lift.emplace(LIFT::DRIVER_A::DRIVER_PINS, LIFT::DRIVER_A::ENCODER_ID, LIFT::DRIVER_A::GROUP_ID,
                 LIFT::DRIVER_B::DRIVER_PINS, LIFT::DRIVER_B::ENCODER_ID, LIFT::DRIVER_B::GROUP_ID,
                 LIFT::CURRENT_SENSE, LIFT::FAULT, &bus);

    packet_manager.emplace(sim);
    lift_group.emplace(bank_group::LIFT, lift.value());
    lift_left_group.emplace(encoder_group::LIFT_L, lift->get_driver_A());
    lift_right_group.emplace(encoder_group::LIFT_R, lift->get_driver_B());
    packet_manager->add_group(lift_group.value());
    packet_manager->add_group(lift_left_group.value());
    packet_manager->add_group(lift_right_group.value());

    bus.begin(kSimEncoders);
    start_bank_control_task({&lift.value(), nullptr, nullptr});

    printf("can_sim: read-only, lift bank held at zero, no commands sent\n");
    printf("  L = driver A (h-bridge 1, GPIO %d/%d) with %s (bus 1, slave 1)\n", LIFT::DRIVER_A::DRIVER_PINS.first,
           LIFT::DRIVER_A::DRIVER_PINS.second, to_string(LIFT::DRIVER_A::ENCODER_ID));
    printf("  R = driver B (h-bridge 2, GPIO %d/%d) with %s (bus 2, slave 1)\n", LIFT::DRIVER_B::DRIVER_PINS.first,
           LIFT::DRIVER_B::DRIVER_PINS.second, to_string(LIFT::DRIVER_B::ENCODER_ID));
}

void loop()
{
    static uint32_t last_print_ms = 0;
    const uint32_t now = encoder_bus().now_ms();

    // Stands in for loop() in main.cpp: caches angles for GET_ANGLE, runs CAN.
    lift->monitor_and_move();
    packet_manager->handle();

    if (now - last_print_ms >= kPrintPeriodMs)
    {
        last_print_ms = now;
        print_status(now);
    }

    delay(1);
}
