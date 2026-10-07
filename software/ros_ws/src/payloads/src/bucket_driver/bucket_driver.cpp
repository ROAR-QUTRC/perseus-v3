#include "bucket_driver/bucket_driver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <hi_can_address.hpp>
#include <hi_can_parameter.hpp>
#include <stdexcept>

namespace
{
    using namespace hi_can;  // NOLINT

    namespace bucket_addr = addressing::excavation::bucket::controller;
    namespace bucket_param = parameters::excavation::bucket::controller;

    /// @brief Address of the bucket controller board itself.
    const addressing::standard_address_t DEVICE_ADDRESS{
        addressing::excavation::SYSTEM_ID,
        addressing::excavation::bucket::SUBSYSTEM_ID,
        bucket_addr::DEVICE_ID};

    /// @brief Banks in the order teleop sends their velocities.
    constexpr std::array<bucket_addr::bank_group, 3> BANKS{
        bucket_addr::bank_group::LIFT,
        bucket_addr::bank_group::TILT,
        bucket_addr::bank_group::JAWS,
    };

    /// @brief Human-readable bank names, for log messages only.
    constexpr std::array<const char*, 3> BANK_NAMES{"lift", "tilt", "jaws"};

    addressing::flagged_address_t set_speed_address(
        const bucket_addr::bank_group& bank)
    {
        return static_cast<addressing::flagged_address_t>(
            addressing::standard_address_t{
                DEVICE_ADDRESS,
                static_cast<uint8_t>(bank),
                static_cast<uint8_t>(bucket_addr::bank_parameter::SET_SPEED)});
    }
}  // namespace

BucketDriver::BucketDriver(const rclcpp::NodeOptions& options)
    : Node("bucket_driver", options)
{
    _max_actuator_speed = this->declare_parameter("max_actuator_speed", 0.1);
    if (_max_actuator_speed <= 0.0)
        throw std::invalid_argument("max_actuator_speed must be positive");

    // Operator override. The deadband keeps stick drift from taking the bucket
    // away from autonomy; anything under it is below the ~10% duty the actuators
    // need to move anyway.
    _override_deadband = this->declare_parameter("override_deadband", 0.1);
    if (_override_deadband <= 0.0 || _override_deadband >= 1.0)
        throw std::invalid_argument("override_deadband must be between 0 and 1");
    _override_controllers = this->declare_parameter<std::vector<std::string>>(
        "override_controllers",
        {"bucket_trajectory_controller", "bucket_lift_controller",
         "bucket_tilt_controller", "bucket_jaw_controller"});
    _rearm_controller = this->declare_parameter<std::string>(
        "rearm_controller", "bucket_trajectory_controller");
    const auto controller_manager =
        this->declare_parameter<std::string>("controller_manager", "/controller_manager");
    _list_client = this->create_client<ListControllers>(controller_manager + "/list_controllers");
    _switch_client =
        this->create_client<SwitchController>(controller_manager + "/switch_controller");
    // Latched, so anything that starts later (MoveBucket, the CLI) sees it at once.
    _override_publisher = this->create_publisher<std_msgs::msg::Bool>(
        "/bucket/operator_override", rclcpp::QoS(1).reliable().transient_local());
    _set_override(false);
    _rearm_service = this->create_service<Trigger>(
        "/bucket/rearm",
        std::bind(&BucketDriver::_rearm_callback, this, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3));

    _can_interface =
        hi_can::RawCanInterface(this->declare_parameter("can_bus", "can0"));

    _actuator_subscription =
        this->create_subscription<actuator_msgs::msg::Actuators>(
            "bucket_actuators", 10,
            std::bind(&BucketDriver::_actuator_callback, this,
                      std::placeholders::_1));

    _transmit_timer = this->create_timer(
        TRANSMIT_INTERVAL, std::bind(&BucketDriver::_transmit_callback, this));

    RCLCPP_INFO(this->get_logger(),
                "Bucket driver initialized - full duty at %.3f, transmitting every %ldms",
                _max_actuator_speed,
                static_cast<long>(TRANSMIT_INTERVAL.count()));
}

BucketDriver::~BucketDriver()
{
    // Stop the bucket on a clean shutdown rather than leaving it to the
    // firmware watchdog, which would keep driving for up to 200ms.
    try
    {
        _write_speeds({});
    }
    catch (const std::exception& e)
    {
        // Nothing useful to do about it this late - the firmware watchdog is
        // still behind us.
        RCLCPP_ERROR(this->get_logger(), "Failed to zero actuators on shutdown: %s",
                     e.what());
    }
}

void BucketDriver::_actuator_callback(
    const actuator_msgs::msg::Actuators::SharedPtr msg)
{
    // The teleop configs still map a fourth "rotate" axis after lift, tilt and
    // jaws. This bucket has no bank for it, so extra entries are ignored.
    if (msg->velocity.size() < ACTUATOR_COUNT)
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "Expected at least %zu actuator velocities (lift, tilt, jaws), got %zu - ignoring",
                              ACTUATOR_COUNT, msg->velocity.size());
        return;
    }

    // A non-finite velocity would make the clamp-and-cast in _write_speeds
    // undefined, so reject the whole message rather than commanding garbage.
    if (!std::all_of(msg->velocity.begin(),
                     msg->velocity.begin() + ACTUATOR_COUNT,
                     [](double v)
                     { return std::isfinite(v); }))
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "Actuator velocities contain a non-finite value - ignoring");
        return;
    }

    std::copy_n(msg->velocity.begin(), ACTUATOR_COUNT, _speeds.begin());
    _last_command_time = this->now();
    _timed_out = false;

    if (!_override && !_override_requested && _above_deadband(_speeds))
        _begin_override();
}

bool BucketDriver::_above_deadband(const std::array<double, ACTUATOR_COUNT>& speeds) const
{
    const double threshold = _override_deadband * _max_actuator_speed;
    return std::any_of(speeds.begin(), speeds.end(),
                       [threshold](double v)
                       { return std::abs(v) > threshold; });
}

void BucketDriver::_begin_override()
{
    _override_requested = this->now();
    RCLCPP_WARN(this->get_logger(), "Operator override: taking the bucket from autonomy");

    if (!_list_client->service_is_ready() || !_switch_client->service_is_ready())
    {
        // Nothing to take it from (bench teleop, or controller_manager down). The
        // timeout in _transmit_callback hands the operator the bucket.
        return;
    }

    // Only the ones actually active: deactivating one that isn't loaded makes the
    // whole switch fail.
    _list_client->async_send_request(
        std::make_shared<ListControllers::Request>(),
        [this](rclcpp::Client<ListControllers>::SharedFuture future)
        {
            auto request = std::make_shared<SwitchController::Request>();
            for (const auto& controller : future.get()->controller)
            {
                if (controller.state == "active" &&
                    std::find(_override_controllers.begin(), _override_controllers.end(),
                              controller.name) != _override_controllers.end())
                    request->deactivate_controllers.push_back(controller.name);
            }
            if (request->deactivate_controllers.empty())
            {
                _set_override(true);
                return;
            }
            request->strictness = SwitchController::Request::BEST_EFFORT;
            request->activate_asap = true;
            _switch_client->async_send_request(
                request,
                [this, names = request->deactivate_controllers](
                    rclcpp::Client<SwitchController>::SharedFuture switched)
                {
                    if (!switched.get()->ok)
                        RCLCPP_ERROR(this->get_logger(),
                                     "Operator override: controller_manager did not confirm "
                                     "deactivating the bucket controllers");
                    else
                        for (const auto& name : names)
                            RCLCPP_WARN(this->get_logger(), "Operator override: deactivated %s",
                                        name.c_str());
                    _set_override(true);
                });
        });
}

void BucketDriver::_set_override(bool active)
{
    _override = active;
    if (active)
        _override_requested.reset();
    std_msgs::msg::Bool msg;
    msg.data = active;
    _override_publisher->publish(msg);
}

void BucketDriver::_rearm_callback(rclcpp::Service<Trigger>::SharedPtr service,
                                   std::shared_ptr<rmw_request_id_t> header,
                                   std::shared_ptr<Trigger::Request> /*request*/)
{
    const auto reply = [service, header](bool success, const std::string& message)
    {
        Trigger::Response response;
        response.success = success;
        response.message = message;
        service->send_response(*header, response);
    };

    if (!_timed_out && _above_deadband(_speeds))
    {
        reply(false, "a stick is still held - let go of the bucket first");
        return;
    }
    if (!_switch_client->service_is_ready())
    {
        reply(false, "controller_manager is not available");
        return;
    }

    // STRICT: if it cannot come up (stale encoders, a joint out of range - see
    // BucketHardware::prepare_command_mode_switch) the operator keeps the bucket.
    // It starts from the current encoder angles, so the bucket holds where the
    // operator left it rather than going back to autonomy's last target.
    auto request = std::make_shared<SwitchController::Request>();
    request->activate_controllers = {_rearm_controller};
    request->strictness = SwitchController::Request::STRICT;
    request->activate_asap = true;
    _switch_client->async_send_request(
        request,
        [this, reply](rclcpp::Client<SwitchController>::SharedFuture future)
        {
            if (!future.get()->ok)
            {
                reply(false, "could not activate " + _rearm_controller +
                                 " - see the controller_manager log");
                return;
            }
            _set_override(false);
            RCLCPP_INFO(this->get_logger(), "Re-armed: %s has the bucket",
                        _rearm_controller.c_str());
            reply(true, _rearm_controller + " active");
        });
}

void BucketDriver::_transmit_callback()
{
    const bool stale =
        !_last_command_time.has_value() ||
        ((this->now() - *_last_command_time) > rclcpp::Duration(ACTUATOR_TIMEOUT));

    if (stale && !_timed_out)
    {
        RCLCPP_WARN(this->get_logger(),
                    "No actuator command for %ldms - zeroing speeds",
                    static_cast<long>(ACTUATOR_TIMEOUT.count()));
        _speeds.fill(0.0);
        _timed_out = true;
    }

    if (_override_requested && (this->now() - *_override_requested) >
                                   rclcpp::Duration(OVERRIDE_SWITCH_TIMEOUT))
    {
        RCLCPP_WARN(this->get_logger(),
                    "Operator override: no answer from controller_manager in %ldms - "
                    "taking the bucket anyway",
                    static_cast<long>(OVERRIDE_SWITCH_TIMEOUT.count()));
        _set_override(true);
    }

    // Sent unconditionally, including while zeroed: a steady stream of zeros
    // keeps the banks in a known state and means a gap on the bus unambiguously
    // signals that this node has died. Zeros sit inside the firmware's deadband,
    // so they never disturb a position the trajectory controller is holding;
    // real speeds go out only once the operator has the bucket.
    _write_speeds(_override ? _speeds : std::array<double, ACTUATOR_COUNT>{});
}

void BucketDriver::_write_speeds(
    const std::array<double, ACTUATOR_COUNT>& speeds)
{
    // speed_t's int16 is an open-loop duty cycle, not a physical speed - the
    // firmware maps the full int16 range across its PWM range. So pick the
    // velocity which should mean "flat out" and scale linearly to that.
    const double conversion_factor =
        static_cast<double>(INT16_MAX) / _max_actuator_speed;

    for (size_t i = 0; i < ACTUATOR_COUNT; i++)
    {
        const auto duty = static_cast<int16_t>(
            std::clamp(speeds[i] * conversion_factor,
                       static_cast<double>(INT16_MIN),
                       static_cast<double>(INT16_MAX)));

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "%s: %+.3f -> %+d", BANK_NAMES[i], speeds[i], duty);

        _can_interface.transmit(Packet(set_speed_address(BANKS[i]),
                                       bucket_param::speed_t{duty}.serialize_data()));
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    int exit_code = 0;
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
        exit_code = 1;
    }

    // Has to run on the failure path too - returning straight out of the catch
    // leaves the context alive into static destruction, which segfaults on exit.
    rclcpp::shutdown();
    return exit_code;
}
