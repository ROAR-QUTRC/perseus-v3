#include "bucket_driver/bucket_driver.hpp"

#include <algorithm>
#include <cstdint>
#include <hi_can_address.hpp>

BucketDriver::BucketDriver(const rclcpp::NodeOptions& options)
    : Node("bucket_driver", options)
{
    _can_interface =
        hi_can::RawCanInterface(this->declare_parameter("can_bus", "can0"));
    _kill_timer = this->create_timer(
        ACTUATOR_TIMEOUT, std::bind(&BucketDriver::_timeout_callback, this));
    _actuator_subscription =
        this->create_subscription<actuator_msgs::msg::Actuators>(
            "bucket_actuators", 10,
            std::bind(&BucketDriver::_actuator_callback, this,
                      std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Bucket driver node initialized");
}

void BucketDriver::_actuator_callback(
    const actuator_msgs::msg::Actuators::SharedPtr msg)
{
    constexpr size_t ACTUATOR_COUNT = 4;
    if ((msg->velocity.size() != ACTUATOR_COUNT) ||
        (msg->normalized.size() != 1))
    {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Received actuator message with incorrect number of actuators");
        return;
    }

    _kill_timer->reset();
    _timed_out = false;
    _write_actuators(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
}
void BucketDriver::_timeout_callback()
{
    if (!_timed_out)
    {
        _write_actuators(0, 0, 0);
        _timed_out = true;
    }
}

void BucketDriver::_write_actuators(double lift, double tilt, double jaws)
{
    constexpr double ACTUATOR_MAX_SPEED = 0.1;  // m/s
    constexpr double LINEAR_CONVERSION_FACTOR = INT16_MAX / ACTUATOR_MAX_SPEED;

    // actuator speeds -> PWM values
    const int16_t lift_pwm = static_cast<int16_t>(std::clamp(
        lift * LINEAR_CONVERSION_FACTOR, static_cast<double>(INT16_MIN),
        static_cast<double>(INT16_MAX)));
    const int16_t tilt_pwm = static_cast<int16_t>(std::clamp(
        tilt * LINEAR_CONVERSION_FACTOR, static_cast<double>(INT16_MIN),
        static_cast<double>(INT16_MAX)));
    const int16_t jaws_pwm = static_cast<int16_t>(std::clamp(
        jaws * LINEAR_CONVERSION_FACTOR, static_cast<double>(INT16_MIN),
        static_cast<double>(INT16_MAX)));

    using namespace hi_can;
    using namespace addressing;
    using namespace addressing::excavation;
    using namespace addressing::excavation::bucket;
    using namespace addressing::excavation::bucket::controller;
    using namespace addressing::excavation::bucket::controller;
    using namespace parameters::excavation::bucket::controller;

    const standard_address_t base_address{SYSTEM_ID, SUBSYSTEM_ID, DEVICE_ID};

    _can_interface.transmit(Packet(
        static_cast<flagged_address_t>(standard_address_t{
            base_address, static_cast<uint8_t>(group::LIFT), static_cast<uint8_t>(bank_parameter::SPEED)}),
        speed_t{lift_pwm}.serialize_data()));

    _can_interface.transmit(Packet(
        static_cast<flagged_address_t>(standard_address_t{
            base_address, static_cast<uint8_t>(group::TILT), static_cast<uint8_t>(bank_parameter::SPEED)}),
        speed_t{tilt_pwm}.serialize_data()));

    _can_interface.transmit(Packet(
        static_cast<flagged_address_t>(standard_address_t{
            base_address, static_cast<uint8_t>(group::JAWS), static_cast<uint8_t>(bank_parameter::SPEED)}),
        speed_t{jaws_pwm}.serialize_data()));
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<BucketDriver>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting bucket driver node");
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error running bucket driver: %s",
                     e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
