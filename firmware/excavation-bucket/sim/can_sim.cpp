// can_sim.cpp
//
// Bench entry point for env:can_sim, built instead of src/main.cpp. Replaces
// the TWAI bus with an in-memory interface that plays the part of ROS and runs
// the lift bank only. After discovery it sends SET_POSITION kTargetDegrees and
// prints status on the console UART while the bank drives there and holds.
//
// Only lift_left is polled, so both actuators follow it: the two lift encoders
// haven't been checked against each other (sign, offset) yet.
//
// This drives the real lift motors.

#include <Arduino.h>

#include <cmath>
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
namespace params = hi_can::parameters::excavation::bucket::controller;

namespace
{
    constexpr gpio_num_t NSLEEP = GPIO_NUM_40;

    constexpr float kTargetDegrees = 350.0f;

    constexpr uint32_t kStallMs = 3000;  // stop if driving but the angle hasn't moved kStallDegrees
    constexpr float kStallDegrees = 0.5f;
    constexpr uint32_t kMoveTimeoutMs = 120000;

    constexpr uint32_t kPrintPeriodMs = 250;

    constexpr EncoderId kFeedback = LIFT::DRIVER_A::ENCODER_ID;
    constexpr uint8_t kSimEncoders = encoder_bit(kFeedback);

    constexpr standard_address_t DEVICE_ADDRESS{
        excavation::SYSTEM_ID,
        excavation::bucket::SUBSYSTEM_ID,
        excavation::bucket::controller::DEVICE_ID,
    };

    enum class Phase
    {
        Discovery,
        Moving,
        Holding,
        Stopped,
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

        std::optional<Packet> receive(bool) override
        {
            while (!rx_.empty())
            {
                const Packet packet = rx_.front();
                rx_.pop_front();
                if (!address_matches_filters(packet.get_address()))
                    continue;
                if (_receive_callback)
                    _receive_callback(packet);
                return packet;
            }
            return std::nullopt;
        }

        void inject(const Packet& packet) { rx_.push_back(packet); }

    private:
        static constexpr size_t kMaxTx = 32;  // periodic reports nobody reads
        std::deque<Packet> rx_;
        std::deque<Packet> tx_;
    };

    SimInterface sim;
    std::optional<PacketManager> packet_manager;
    std::optional<MotorBank> lift;
    std::optional<MotorBankParameterGroup> lift_group;
    std::optional<MotorParameterGroup> lift_left_group;

    const char* phase_name(Phase phase)
    {
        switch (phase)
        {
        case Phase::Discovery:
            return "discovery";
        case Phase::Moving:
            return "moving";
        case Phase::Holding:
            return "holding";
        case Phase::Stopped:
            return "stopped";
        }
        return "?";
    }

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

    void send_position(float degrees)
    {
        const flagged_address_t address = static_cast<flagged_address_t>(standard_address_t{
            DEVICE_ADDRESS, static_cast<uint8_t>(bank_group::LIFT), static_cast<uint8_t>(bank_parameter::SET_POSITION)});
        const auto units = static_cast<int16_t>(std::lround(degrees * kPositionUnitsPerDegree));
        sim.inject(Packet(address, params::position_t{units}.serialize_data()));
    }

    void print_status(uint32_t now, Phase phase, std::optional<float> angle)
    {
        const MotorBank::Status status = lift->get_status();
        const bool position = status.mode == MotorBank::ControlMode::Position;
        EncoderReading reading;
        encoder_bus().get(kFeedback, &reading);

        printf("%7.2fs  %-9s %s", now / 1000.0f, phase_name(phase), position ? "POS" : "VEL");
        if (position)
            printf(" tgt %5.1f", status.target_position / kPositionUnitsPerDegree);
        else
            printf(" spd %4.0f%%", MotorBank::to_percent(status.speed));
        printf("  out %4.0f%%/%4.0f%%  angle ", MotorBank::to_percent(status.output_a),
               MotorBank::to_percent(status.output_b));
        if (angle)
        {
            printf("%7.2f", *angle);
            if (position)
                printf(" err %+7.2f",
                       MotorBank::angle_difference(status.target_position / kPositionUnitsPerDegree, *angle));
        }
        else if (!reading.present)
            printf("not found");
        else if (!reading.angle_valid)
            printf("invalid");
        else
            printf("stale %lums", static_cast<unsigned long>(reading.angle_age_ms(now)));
        printf("  (%s %lu/%lu)\n", link_name(reading.link), static_cast<unsigned long>(reading.stats.ok),
               static_cast<unsigned long>(reading.stats.total));
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
    packet_manager->add_group(lift_group.value());
    packet_manager->add_group(lift_left_group.value());

    bus.begin(kSimEncoders);
    start_bank_control_task({&lift.value(), nullptr, nullptr});

    printf("can_sim: lift to %.0f deg at %.0f%%, within %.0f deg, feedback %s for both actuators. Discovery first (~24 s)\n",
           kTargetDegrees, MotorBank::kPositionSpeed, MotorBank::kHoldWindow, to_string(kFeedback));
}

void loop()
{
    static Phase phase = Phase::Discovery;
    static uint32_t move_start_ms = 0;
    static uint32_t last_print_ms = 0;
    static float progress_angle = 0.0f;
    static uint32_t progress_ms = 0;

    const uint32_t now = encoder_bus().now_ms();
    const std::optional<float> angle = MotorBank::encoder_degrees(kFeedback, now);

    if (phase == Phase::Discovery && encoder_bus().ready() && angle)
    {
        printf("\n--- moving to %.0f: shortest way from %.2f is %+.2f deg\n", kTargetDegrees, *angle,
               MotorBank::angle_difference(kTargetDegrees, *angle));
        send_position(kTargetDegrees);
        phase = Phase::Moving;
        move_start_ms = now;
        progress_angle = *angle;
        progress_ms = now;
    }
    else if ((phase == Phase::Moving || phase == Phase::Holding) && angle)
    {
        const MotorBank::Status status = lift->get_status();
        const bool driving = status.output_a != 0 || status.output_b != 0;

        if (!driving || std::fabs(MotorBank::angle_difference(*angle, progress_angle)) > kStallDegrees)
        {
            progress_angle = *angle;
            progress_ms = now;
        }

        if (driving && now - progress_ms >= kStallMs)
        {
            lift->stop();
            phase = Phase::Stopped;
            printf("--- STOP: stalled at %.2f, driving but not moving (end stop?)\n", *angle);
        }
        else if (phase == Phase::Moving && !driving)
        {
            phase = Phase::Holding;
            printf("--- reached %.0f (angle %.2f) in %.1f s, holding\n", kTargetDegrees, *angle,
                   (now - move_start_ms) / 1000.0f);
        }
        else if (phase == Phase::Holding && driving)
        {
            phase = Phase::Moving;
            printf("--- drifted to %.2f, homing back in\n", *angle);
        }
        else if (phase == Phase::Moving && now - move_start_ms >= kMoveTimeoutMs)
        {
            lift->stop();
            phase = Phase::Stopped;
            printf("--- STOP: didn't reach %.0f within %lu s (angle %.2f)\n", kTargetDegrees,
                   static_cast<unsigned long>(kMoveTimeoutMs / 1000), *angle);
        }
    }

    // Stands in for loop() in main.cpp: caches angles for GET_ANGLE, runs CAN.
    lift->monitor_and_move();
    packet_manager->handle();

    // Quiet during discovery (~9 s) so its encoder_bus log lines stay readable.
    if (phase != Phase::Discovery && now - last_print_ms >= kPrintPeriodMs)
    {
        last_print_ms = now;
        print_status(now, phase, angle);
    }

    delay(1);
}
