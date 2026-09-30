#include "bucket_hardware/bucket_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace payloads
{

    namespace
    {
        rclcpp::Logger logger() { return rclcpp::get_logger("payload"); }
    }  // namespace

    bool BucketHardware::parse_axis(const std::string& value, Axis& out)
    {
        if (value == "lift")
        {
            out = Axis::LIFT;
            return true;
        }
        if (value == "tilt")
        {
            out = Axis::TILT;
            return true;
        }
        if (value == "jaws")
        {
            out = Axis::JAWS;
            return true;
        }
        return false;
    }

    bool BucketHardware::parse_side(const std::string& value, Side& out)
    {
        if (value == "left")
        {
            out = Side::LEFT;
            return true;
        }
        if (value == "right")
        {
            out = Side::RIGHT;
            return true;
        }
        return false;
    }

    hardware_interface::CallbackReturn BucketHardware::on_init(
        const hardware_interface::HardwareInfo& info)
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Optional top-level hardware parameter: <param name="can_interface">can0</param>
        auto it = info_.hardware_parameters.find("can_interface");
        if (it != info_.hardware_parameters.end())
        {
            can_interface_name_ = it->second;
        }

        joints_.clear();
        joints_.reserve(info_.joints.size());

        for (const auto& joint_info : info_.joints)
        {
            JointHandle joint;
            joint.name = joint_info.name;

            // Each joint must declare which physical actuator it drives, e.g.:
            //   <param name="axis">lift</param>
            //   <param name="side">left</param>
            auto axis_it = joint_info.parameters.find("axis");
            auto side_it = joint_info.parameters.find("side");
            if (axis_it == joint_info.parameters.end() || side_it == joint_info.parameters.end())
            {
                RCLCPP_ERROR(
                    logger(), "Joint '%s' is missing required 'axis' and/or 'side' parameters",
                    joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (!parse_axis(axis_it->second, joint.axis) || !parse_side(side_it->second, joint.side))
            {
                RCLCPP_ERROR(
                    logger(), "Joint '%s' has an invalid axis ('%s') or side ('%s')",
                    joint.name.c_str(), axis_it->second.c_str(), side_it->second.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }

            // Optional firmware -> URDF frame calibration; defaults are identity.
            try
            {
                if (auto p = joint_info.parameters.find("offset_deg"); p != joint_info.parameters.end())
                {
                    joint.offset_deg = std::stod(p->second);
                }
                if (auto p = joint_info.parameters.find("direction"); p != joint_info.parameters.end())
                {
                    joint.direction = std::stod(p->second);
                }
            }
            catch (const std::exception&)
            {
                RCLCPP_ERROR(logger(), "Joint '%s' has a non-numeric offset_deg or direction",
                             joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (joint.direction != 1.0 && joint.direction != -1.0)
            {
                RCLCPP_ERROR(logger(), "Joint '%s' direction must be 1 or -1", joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }

            // Every joint gets all 3 state interfaces. Commands are bank-level
            // (CAN is setup to only accepts one setpoint per axis), so exactly one joint
            // per axis may declare the 2 command interfaces.
            if (joint_info.state_interfaces.size() != 3)
            {
                RCLCPP_ERROR(
                    logger(), "Joint '%s' must declare exactly 3 state interfaces (position, velocity, effort)",
                    joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
            if (joint_info.command_interfaces.size() == 1)
            {
                joint.has_command = true;
            }
            else if (joint_info.command_interfaces.size() != 0)
            {
                RCLCPP_ERROR(
                    logger(),
                    "Joint '%s' must declare either 0 command interfaces (state-only) or exactly 1 "
                    "(position) - got %zu",
                    joint.name.c_str(), joint_info.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            joints_.push_back(joint);
        }

        for (uint8_t axis_idx = 0; axis_idx < kNumAxes; ++axis_idx)
        {
            const auto axis = static_cast<Axis>(axis_idx);
            const auto commandable = std::count_if(
                joints_.begin(), joints_.end(),
                [axis](const JointHandle& j)
                { return j.axis == axis && j.has_command; });
            if (commandable != 1)
            {
                RCLCPP_ERROR(
                    logger(),
                    "Exactly one joint must declare command interfaces per axis; found %ld for "
                    "axis %d",
                    commandable, axis_idx);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        can_ = std::make_unique<CanBoardInterface>();

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn BucketHardware::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        if (!can_->connect(can_interface_name_))
        {
            RCLCPP_ERROR(logger(), "Failed to connect to CAN interface '%s'", can_interface_name_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Both callbacks fire synchronously from within can_->poll(), called at
        // the top of read() every control cycle

        // Per-encoder angle -> that joint's position state.
        can_->register_encoder_callback(
            [this](Axis axis, Side side, const EncoderState& state)
            {
                for (auto& joint : joints_)
                {
                    if (joint.axis == axis && joint.side == side)
                    {
                        joint.state_position = to_joint_radians(joint, state.degrees);
                        break;
                    }
                }
            });

        // Per-bank current/fault for both joints of that axis (bank current is
        // not per-side, so it's duplicated onto both joints' "effort" interface).
        can_->register_bank_callback(
            [this](Axis axis, const BankState& state)
            {
                for (auto& joint : joints_)
                {
                    if (joint.axis == axis)
                    {
                        joint.state_current = state.current;
                        // TODO: surface state.fault / state.average_degrees (e.g.
                        // compare average_degrees against the two encoders' average as a
                        // skew check) somewhere - diagnostics publisher, or fail read().
                    }
                }
            });

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn BucketHardware::on_cleanup(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        can_->disconnect();
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn BucketHardware::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // Wait for a real reading on every joint before any controller can
        // activate: a controller that starts from a zero state holds, and so
        // commands, the URDF zero pose.
        const auto deadline = std::chrono::steady_clock::now() + ACTIVATE_TIMEOUT;
        const auto all_fresh = [this]()
        {
            return std::all_of(joints_.begin(), joints_.end(), [this](const JointHandle& j)
                               { return !can_->get_last_encoder_state(j.axis, j.side).stale; });
        };
        while (can_->poll(), !all_fresh())
        {
            if (std::chrono::steady_clock::now() > deadline)
            {
                for (const auto& joint : joints_)
                {
                    if (can_->get_last_encoder_state(joint.axis, joint.side).stale)
                    {
                        RCLCPP_ERROR(logger(), "No encoder data for joint '%s' on %s",
                                     joint.name.c_str(), can_interface_name_.c_str());
                    }
                }
                return hardware_interface::CallbackReturn::ERROR;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // A reading well outside the URDF limits means the firmware zero or the
        // direction param is wrong, and the limits would clamp commands in the
        // wrong frame. Refuse rather than move.
        for (const auto& joint : joints_)
        {
            const auto limits = info_.limits.find(joint.name);
            if (limits == info_.limits.end() || !limits->second.has_position_limits)
            {
                continue;
            }
            const double margin = LIMIT_MARGIN_DEG * M_PI / 180.0;
            if (joint.state_position < limits->second.min_position - margin ||
                joint.state_position > limits->second.max_position + margin)
            {
                RCLCPP_ERROR(logger(),
                             "Joint '%s' reads %.1f deg, outside its limits [%.1f, %.1f] deg - "
                             "check the firmware zero and the offset_deg/direction params",
                             joint.name.c_str(), joint.state_position * 180.0 / M_PI,
                             limits->second.min_position * 180.0 / M_PI,
                             limits->second.max_position * 180.0 / M_PI);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        // Seed commands with current state so the first write() doesn't jerk the
        // actuators toward a stale/zero setpoint. perform_command_mode_switch()
        // re-seeds on each claim too, since activation and claiming are separate
        // events and state_position moves in between.
        for (auto& joint : joints_)
        {
            joint.command_position = joint.state_position;
            joint.previous_position = joint.state_position;
            joint.state_velocity = 0.0;
        }
        active_ = true;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn BucketHardware::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // Stop every bank explicitly. The firmware watchdogs a stale SET_SPEED
        // but not a stale SET_POSITION, so without this the bucket would keep
        // driving toward its last setpoint after the controllers stop.
        active_ = false;
        for (uint8_t axis_idx = 0; axis_idx < kNumAxes; ++axis_idx)
        {
            can_->stop_axis(static_cast<Axis>(axis_idx));
        }
        for (auto& joint : joints_)
        {
            joint.command_claimed = false;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type BucketHardware::prepare_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces)
    {
        // Position is the only command interface this hardware exports, so any
        // claim controller_manager can construct is one we can honour. Validation
        // that only one joint per axis is commandable happened back in on_init().
        (void)start_interfaces;
        (void)stop_interfaces;
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type BucketHardware::perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces)
    {
        // controller_manager hands us interfaces as "<joint>/<interface>".
        const auto matches = [](const std::string& full_name, const JointHandle& joint)
        {
            return full_name == joint.name + "/" + hardware_interface::HW_IF_POSITION;
        };

        for (auto& joint : joints_)
        {
            for (const auto& name : stop_interfaces)
            {
                if (matches(name, joint))
                {
                    joint.command_claimed = false;
                    // Stop the bank rather than leaving it acting on the setpoint
                    // it was last given. Nothing else will: the firmware does not
                    // watchdog position commands.
                    can_->stop_axis(joint.axis);
                }
            }
            for (const auto& name : start_interfaces)
            {
                if (matches(name, joint))
                {
                    // Start from where the bucket actually is, so the first
                    // write() cannot jerk it toward a stale setpoint.
                    joint.command_position = joint.state_position;
                    joint.command_claimed = true;
                }
            }
        }
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> BucketHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.reserve(joints_.size() * 3);

        for (auto& joint : joints_)
        {
            state_interfaces.emplace_back(
                joint.name, hardware_interface::HW_IF_POSITION, &joint.state_position);
            state_interfaces.emplace_back(
                joint.name, hardware_interface::HW_IF_VELOCITY, &joint.state_velocity);
            state_interfaces.emplace_back(
                joint.name, hardware_interface::HW_IF_EFFORT, &joint.state_current);
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> BucketHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.reserve(kNumAxes);

        // Only the one commandable joint per axis exports command interfaces -
        // the other side declared none in the xacro (see on_init()'s validation).
        for (auto& joint : joints_)
        {
            if (!joint.has_command)
            {
                continue;
            }
            command_interfaces.emplace_back(
                joint.name, hardware_interface::HW_IF_POSITION, &joint.command_position);
        }
        return command_interfaces;
    }

    hardware_interface::return_type BucketHardware::read(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
    {
        // hi_can::PacketManager is polled, decoding buffered frames. Fires
        // the encoder/bank callbacks registered in on_configure().
        // Must run before joint state is considered current.
        can_->poll();

        const double dt = period.seconds();
        for (auto& joint : joints_)
        {
            if (dt > 0.0)
            {
                const double sample = (joint.state_position - joint.previous_position) / dt;
                joint.state_velocity += VELOCITY_FILTER_ALPHA * (sample - joint.state_velocity);
            }
            joint.previous_position = joint.state_position;
        }

        // Refuse to keep running on bad feedback. Returning ERROR makes
        // controller_manager deactivate the controllers, which routes into
        // on_deactivate() and stops the bucket.
        //
        // This matters more than it looks: the firmware watchdogs a stale
        // SET_SPEED but NOT a stale SET_POSITION, so on the position path there
        // is no firmware deadman behind us.
        // on_activate() waits for every encoder, so staleness is only judged once
        // active; read() also runs while inactive.
        const bool settled = active_;

        for (const auto& joint : joints_)
        {
            if (settled && can_->get_last_encoder_state(joint.axis, joint.side).stale)
            {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 1000,
                    "Encoder for joint '%s' is stale - stopping", joint.name.c_str());
                return hardware_interface::return_type::ERROR;
            }
        }
        for (uint8_t axis_idx = 0; axis_idx < kNumAxes; ++axis_idx)
        {
            const auto axis = static_cast<Axis>(axis_idx);
            if (settled && can_->get_last_bank_state(axis).fault)
            {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                                      "Bank %d reports a fault - stopping", axis_idx);
                return hardware_interface::return_type::ERROR;
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type BucketHardware::write(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        // Exactly one command per axis (bank-level control)
        // Find the one commandable joint per axis rather than iterating all 6.
        for (const auto& joint : joints_)
        {
            if (!joint.has_command || !joint.command_claimed)
            {
                // Silence is deliberate when nothing has claimed the interface.
                // Transmitting the last setpoint anyway would keep the firmware's
                // command timer fed - so its watchdog would never fire - and
                // would fight the standalone teleop driver for control mode,
                // since the firmware follows whichever command arrived last.
                continue;
            }
            can_->send_position_command(joint.axis,
                                        to_firmware_degrees(joint, joint.command_position));
        }
        return hardware_interface::return_type::OK;
    }

    double BucketHardware::to_joint_radians(const JointHandle& joint, double firmware_degrees) const
    {
        return joint.direction * wrap_degrees(firmware_degrees - joint.offset_deg) * M_PI / 180.0;
    }

    double BucketHardware::to_firmware_degrees(const JointHandle& joint, double joint_radians) const
    {
        return joint.offset_deg + joint.direction * joint_radians * 180.0 / M_PI;
    }

}  // namespace payloads

PLUGINLIB_EXPORT_CLASS(payloads::BucketHardware, hardware_interface::SystemInterface)