#include "bucket_hardware/can_board_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace payload
{

    namespace
    {
        rclcpp::Logger logger() { return rclcpp::get_logger("bucket_hardware"); }
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
            if (joint_info.command_interfaces.size() == 2)
            {
                joint.has_command = true;
            }
            else if (joint_info.command_interfaces.size() != 0)
            {
                RCLCPP_ERROR(
                    logger(),
                    "Joint '%s' must declare either 0 command interfaces (state-only) or exactly 2 "
                    "(position, velocity) - got %zu",
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
                        joint.state_position = state.position;
                        // TODO: differentiate successive position samples for
                        // state_velocity, or leave at 0.0 if not needed.
                        // TODO: surface state.stale somewhere (e.g. fail read() if stale
                        // for too long, or a diagnostics publisher).
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
                        // TODO: surface state.fault / state.average_position (e.g.
                        // compare average_position against the two encoders' average as a
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
        // Seed commands with current state so the first write() doesn't jerk the
        // actuators toward a stale/zero setpoint.
        for (auto& joint : joints_)
        {
            joint.command_position = joint.state_position;
            joint.command_velocity = 0.0;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn BucketHardware::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // TODO: add a "hold" or "disable" command here so the board
        // doesn't keep driving actuators toward the last commanded setpoint after
        // controllers stop updating.
        return hardware_interface::CallbackReturn::SUCCESS;
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
        command_interfaces.reserve(kNumAxes * 2);

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
            command_interfaces.emplace_back(
                joint.name, hardware_interface::HW_IF_VELOCITY, &joint.command_velocity);
        }
        return command_interfaces;
    }

    hardware_interface::return_type BucketHardware::read(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        // hi_can::PacketManager is polled, decoding buffered frames. Fires
        // the encoder/bank callbacks registered in on_configure().
        // Must run before joint state is considered current.
        can_->poll();

        // Example staleness check, modify to match final implementation:
        // for (auto & joint : joints_) {
        //   if (can_->get_last_encoder_state(joint.axis, joint.side).stale) {
        //     return hardware_interface::return_type::ERROR;
        //   }
        // }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type BucketHardware::write(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        // Exactly one command per axis (bank-level control)
        // Find the one commandable joint per axis rather than iterating all 6.
        for (const auto& joint : joints_)
        {
            if (!joint.has_command)
            {
                continue;
            }
            // Position and velocity controllers are mutually exclusive (see
            // bucket_controllers.yaml), so send whichever the active controller is
            // actually writing. Sending both every cycle would mean the board
            // receives a stale/zero value on whichever interface no controller has
            // claimed.
            // TODO: this assumes only the claimed interface's command is
            // meaningful; if neither controller is active this still sends
            // command_position (usually 0.0 or the last-seeded value from
            // on_activate()) via SET_ANGLE every cycle. Track which interface is
            // actually claimed if not acceptable.
            // TODO: The controller should use position for autonomous operation,
            // and velocity for manual adjustment (moving at a speed vs to a position)
            can_->send_position_command(joint.axis, joint.command_position);
        }
        return hardware_interface::return_type::OK;
    }

}  // namespace payload

PLUGINLIB_EXPORT_CLASS(bucket_hardware::BucketHardware, hardware_interface::SystemInterface)