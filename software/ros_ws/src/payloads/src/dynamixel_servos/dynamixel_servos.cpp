#include "dynamixel_servos/dynamixel_servos.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <pluginlib/class_list_macros.hpp>
#include <sstream>
#include <stdexcept>
#include <string>

#include "dynamixel_controller.hpp"

namespace payloads
{

    DynamixelServos::~DynamixelServos() = default;

    bool DynamixelServos::isMocked(int servo_id) const
    {
        return mock_servo_ids_.count(servo_id) > 0;
    }

    hardware_interface::CallbackReturn DynamixelServos::on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params)
    {
        // Populates the interface maps from the URDF; without it every
        // set_state()/get_command() below fails to find its handle.
        auto ret = hardware_interface::SystemInterface::on_init(params);
        if (ret != hardware_interface::CallbackReturn::SUCCESS)
        {
            return ret;
        }

        auto hw = info_.hardware_parameters.find("device");
        if (hw != info_.hardware_parameters.end())
        {
            device_ = hw->second;
        }
        auto baud = info_.hardware_parameters.find("baud_rate");
        if (baud != info_.hardware_parameters.end())
        {
            baud_rate_ = std::atoi(baud->second.c_str());
        }

        // Comma-separated list of servo IDs to simulate (e.g. "4,5,6").
        auto mock = info_.hardware_parameters.find("mock_servo_ids");
        if (mock != info_.hardware_parameters.end() && !mock->second.empty())
        {
            std::istringstream ss(mock->second);
            std::string token;
            while (std::getline(ss, token, ','))
            {
                mock_servo_ids_.insert(std::atoi(token.c_str()));
            }
            RCLCPP_INFO(get_logger(), "Mocking %zu servo ID(s) (no bus communication)",
                        mock_servo_ids_.size());
        }

        ret = parseJoints();
        if (ret != hardware_interface::CallbackReturn::SUCCESS)
        {
            return ret;
        }

        ret = parseTransmissions();
        if (ret != hardware_interface::CallbackReturn::SUCCESS)
        {
            return ret;
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DynamixelServos::parseJoints()
    {
        joints_.reserve(info_.joints.size());
        for (const auto& joint_info : info_.joints)
        {
            Joint j;
            j.name = joint_info.name;

            auto servo = joint_info.parameters.find("servo_id");
            if (servo == joint_info.parameters.end())
            {
                RCLCPP_ERROR(get_logger(),
                             "Joint '%s' is missing required 'servo_id' parameter",
                             j.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
            j.servo_id = std::atoi(servo->second.c_str());

            auto scale = joint_info.parameters.find("prismatic_scale");
            if (scale != joint_info.parameters.end())
            {
                j.prismatic_scale = std::atof(scale->second.c_str());
                if (j.prismatic_scale != 1.0)
                {
                    j.is_prismatic = true;
                }
            }

            joints_.push_back(std::move(j));
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DynamixelServos::parseTransmissions()
    {
        for (const auto& trans : info_.transmissions)
        {
            if (trans.type == "transmission_interface/SimpleTransmission")
            {
                if (trans.joints.size() != 1)
                {
                    RCLCPP_ERROR(get_logger(),
                                 "SimpleTransmission '%s' must have exactly one joint",
                                 trans.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }
                const auto& j = trans.joints[0];
                auto it =
                    std::find_if(joints_.begin(), joints_.end(), [&](const Joint& joint)
                                 { return joint.name == j.name; });
                if (it == joints_.end())
                {
                    RCLCPP_ERROR(get_logger(),
                                 "SimpleTransmission '%s' references unknown joint '%s'",
                                 trans.name.c_str(), j.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }
                it->has_transmission = true;
                it->joint_reduction = j.mechanical_reduction;
                it->joint_offset = j.offset;
                if (!trans.actuators.empty())
                {
                    it->actuator_reduction = trans.actuators[0].mechanical_reduction;
                }
            }
            else if (trans.type ==
                     "transmission_interface/DifferentialTransmission")
            {
                if (trans.joints.size() != 2 || trans.actuators.size() != 2)
                {
                    RCLCPP_ERROR(get_logger(),
                                 "DifferentialTransmission '%s' must have exactly two "
                                 "joints and two actuators",
                                 trans.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }

                DifferentialGroup diff;

                int j1 = -1, j2 = -1;
                for (const auto& tj : trans.joints)
                {
                    auto it = std::find_if(
                        joints_.begin(), joints_.end(),
                        [&](const Joint& joint)
                        { return joint.name == tj.name; });
                    if (it == joints_.end())
                    {
                        RCLCPP_ERROR(
                            get_logger(),
                            "DifferentialTransmission '%s' references unknown joint '%s'",
                            trans.name.c_str(), tj.name.c_str());
                        return hardware_interface::CallbackReturn::ERROR;
                    }
                    int idx = static_cast<int>(std::distance(joints_.begin(), it));
                    it->in_differential = true;
                    if (tj.role == "joint1")
                    {
                        j1 = idx;
                        diff.jr[0] = tj.mechanical_reduction;
                        diff.off[0] = tj.offset;
                    }
                    else if (tj.role == "joint2")
                    {
                        j2 = idx;
                        diff.jr[1] = tj.mechanical_reduction;
                        diff.off[1] = tj.offset;
                    }
                }
                if (j1 < 0 || j2 < 0)
                {
                    RCLCPP_ERROR(get_logger(),
                                 "DifferentialTransmission '%s' is missing joint1 or "
                                 "joint2 role",
                                 trans.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }
                diff.joint1_index = j1;
                diff.joint2_index = j2;

                for (const auto& ta : trans.actuators)
                {
                    if (ta.role == "actuator1")
                    {
                        diff.ar[0] = ta.mechanical_reduction;
                        diff.actuator1_servo_id = joints_[j1].servo_id;
                    }
                    else if (ta.role == "actuator2")
                    {
                        diff.ar[1] = ta.mechanical_reduction;
                        diff.actuator2_servo_id = joints_[j2].servo_id;
                    }
                }

                differentials_.push_back(std::move(diff));
            }
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    double DynamixelServos::forwardTransform(const Joint& joint,
                                             double actuator_pos) const
    {
        // SimpleTransmission forward: j = a / jr + offset
        double joint_pos =
            joint.has_transmission
                ? actuator_pos / joint.joint_reduction + joint.joint_offset
                : actuator_pos;
        if (joint.is_prismatic)
        {
            joint_pos *= joint.prismatic_scale;
        }
        return joint_pos;
    }

    double DynamixelServos::inverseTransform(const Joint& joint,
                                             double joint_pos) const
    {
        // Prismatic conversion first: metres -> radians in joint space.
        if (joint.is_prismatic)
        {
            joint_pos /= joint.prismatic_scale;
        }
        // SimpleTransmission inverse: a = (j - offset) * jr
        return joint.has_transmission
                   ? (joint_pos - joint.joint_offset) * joint.joint_reduction
                   : joint_pos;
    }

    double DynamixelServos::normalizeAngle(double angle)
    {
        constexpr double tau = 6.28318530717958647692;
        angle = std::fmod(angle + M_PI, tau);
        if (angle < 0.0)
        {
            angle += tau;
        }
        return angle - M_PI;
    }

    double DynamixelServos::normalizeAnglePositive(double angle)
    {
        constexpr double tau = 6.28318530717958647692;
        angle = std::fmod(angle, tau);
        if (angle < 0.0)
        {
            angle += tau;
        }
        return angle;
    }

    hardware_interface::CallbackReturn DynamixelServos::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // Connector throws if the port cannot be opened or the baud rate set;
        // let that surface as a lifecycle failure, not a controller_manager crash.
        try
        {
            controller_ = std::make_unique<DynamixelController>(device_, baud_rate_);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "Could not open Dynamixel bus on %s: %s",
                         device_.c_str(), e.what());
            return hardware_interface::CallbackReturn::FAILURE;
        }

        if (const auto& err = controller_->lowLatencyError(); !err.empty())
        {
            RCLCPP_WARN(get_logger(),
                        "Could not set 1 ms FTDI latency on %s (%s); the 16 ms default "
                        "will hold the control loop well below its update rate",
                        device_.c_str(), err.c_str());
        }

        auto scan_result = controller_->scan();
        if (!scan_result.isSuccess())
        {
            RCLCPP_ERROR(get_logger(), "Dynamixel scan failed: %s",
                         dynamixel::getErrorMessage(scan_result.error()).c_str());
            controller_.reset();
            return hardware_interface::CallbackReturn::FAILURE;
        }

        const auto& found = scan_result.value();
        for (const auto& joint : joints_)
        {
            if (isMocked(joint.servo_id))
            {
                continue;
            }
            if (std::find(found.begin(), found.end(),
                          static_cast<uint8_t>(joint.servo_id)) == found.end())
            {
                RCLCPP_ERROR(get_logger(),
                             "Servo ID %d (joint '%s') was not found on the bus",
                             joint.servo_id, joint.name.c_str());
                controller_.reset();
                return hardware_interface::CallbackReturn::FAILURE;
            }
        }

        // Ensure every real servo is in POSITION mode before reading positions or
        // enabling torque. A servo left in VELOCITY/PWM from a prior session would
        // ignore position commands.
        for (const auto& joint : joints_)
        {
            if (isMocked(joint.servo_id))
            {
                continue;
            }
            auto mode_result =
                controller_->setMode(static_cast<uint8_t>(joint.servo_id),
                                     DynamixelController::Mode::POSITION);
            if (!mode_result.isSuccess())
            {
                RCLCPP_ERROR(get_logger(), "Failed to set POSITION mode on servo %d: %s",
                             joint.servo_id,
                             dynamixel::getErrorMessage(mode_result.error()).c_str());
                controller_.reset();
                return hardware_interface::CallbackReturn::FAILURE;
            }
        }

        auto pos_result = controller_->readPositions();
        if (!pos_result.isSuccess())
        {
            RCLCPP_ERROR(get_logger(), "Initial position read failed: %s",
                         dynamixel::getErrorMessage(pos_result.error()).c_str());
            controller_.reset();
            return hardware_interface::CallbackReturn::FAILURE;
        }
        auto positions = pos_result.value();

        for (int id : mock_servo_ids_)
        {
            mock_positions_[static_cast<uint8_t>(id)] = 0.0;
            positions[static_cast<uint8_t>(id)] = 0.0;
        }

        for (const auto& diff : differentials_)
        {
            double a1 = normalizeAngle(
                positions.at(static_cast<uint8_t>(diff.actuator1_servo_id)));
            double a2 = normalizeAngle(
                positions.at(static_cast<uint8_t>(diff.actuator2_servo_id)));
            double j1 =
                (a1 / diff.ar[0] + a2 / diff.ar[1]) / (2.0 * diff.jr[0]) + diff.off[0];
            double j2 =
                (a1 / diff.ar[0] - a2 / diff.ar[1]) / (2.0 * diff.jr[1]) + diff.off[1];
            set_state(joints_[diff.joint1_index].name + "/position", j1);
            set_state(joints_[diff.joint1_index].name + "/velocity", 0.0);
            set_state(joints_[diff.joint2_index].name + "/position", j2);
            set_state(joints_[diff.joint2_index].name + "/velocity", 0.0);
        }

        for (const auto& joint : joints_)
        {
            if (joint.in_differential)
            {
                continue;
            }
            double actuator_pos =
                normalizeAngle(positions.at(static_cast<uint8_t>(joint.servo_id)));
            double joint_pos = forwardTransform(joint, actuator_pos);
            set_state(joint.name + "/position", joint_pos);
            set_state(joint.name + "/velocity", 0.0);
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DynamixelServos::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        for (const auto& joint : joints_)
        {
            if (isMocked(joint.servo_id))
            {
                continue;
            }
            auto result =
                controller_->enableTorque(static_cast<uint8_t>(joint.servo_id));
            if (!result.isSuccess())
            {
                RCLCPP_ERROR(get_logger(), "Failed to enable torque on servo %d: %s",
                             joint.servo_id,
                             dynamixel::getErrorMessage(result.error()).c_str());
                return hardware_interface::CallbackReturn::FAILURE;
            }
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DynamixelServos::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        for (const auto& joint : joints_)
        {
            if (isMocked(joint.servo_id))
            {
                continue;
            }
            auto result =
                controller_->disableTorque(static_cast<uint8_t>(joint.servo_id));
            if (!result.isSuccess())
            {
                RCLCPP_WARN(get_logger(), "Failed to disable torque on servo %d: %s",
                            joint.servo_id,
                            dynamixel::getErrorMessage(result.error()).c_str());
            }
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type
    DynamixelServos::read(const rclcpp::Time& /*time*/,
                          const rclcpp::Duration& /*period*/)
    {
        auto pos_result = controller_->readPositions();
        if (!pos_result.isSuccess())
        {
            RCLCPP_WARN(get_logger(), "readPositions failed: %s",
                        dynamixel::getErrorMessage(pos_result.error()).c_str());
            return hardware_interface::return_type::OK;
        }
        auto positions = pos_result.value();
        for (const auto& [id, pos] : mock_positions_)
        {
            positions[id] = pos;
        }

        for (const auto& diff : differentials_)
        {
            double a1 = normalizeAngle(
                positions.at(static_cast<uint8_t>(diff.actuator1_servo_id)));
            double a2 = normalizeAngle(
                positions.at(static_cast<uint8_t>(diff.actuator2_servo_id)));
            double j1 =
                (a1 / diff.ar[0] + a2 / diff.ar[1]) / (2.0 * diff.jr[0]) + diff.off[0];
            double j2 =
                (a1 / diff.ar[0] - a2 / diff.ar[1]) / (2.0 * diff.jr[1]) + diff.off[1];
            set_state(joints_[diff.joint1_index].name + "/position", j1);
            set_state(joints_[diff.joint1_index].name + "/velocity", 0.0);
            set_state(joints_[diff.joint2_index].name + "/position", j2);
            set_state(joints_[diff.joint2_index].name + "/velocity", 0.0);
        }

        for (const auto& joint : joints_)
        {
            if (joint.in_differential)
            {
                continue;
            }
            auto it = positions.find(static_cast<uint8_t>(joint.servo_id));
            if (it == positions.end())
            {
                continue;
            }
            double joint_pos = forwardTransform(joint, normalizeAngle(it->second));
            set_state(joint.name + "/position", joint_pos);
            set_state(joint.name + "/velocity", 0.0);
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type
    DynamixelServos::write(const rclcpp::Time& /*time*/,
                           const rclcpp::Duration& /*period*/)
    {
        std::unordered_map<uint8_t, double> targets;

        for (const auto& joint : joints_)
        {
            if (joint.in_differential)
            {
                continue;
            }
            double cmd = get_command(joint.name + "/position");
            // Controllers have not written a command yet on the first few cycles;
            // converting NaN to servo units would throw out_of_range.
            if (!std::isfinite(cmd))
            {
                continue;
            }
            double actuator_pos = inverseTransform(joint, cmd);
            targets[static_cast<uint8_t>(joint.servo_id)] = actuator_pos;
        }

        for (const auto& diff : differentials_)
        {
            double j1 = get_command(joints_[diff.joint1_index].name + "/position");
            double j2 = get_command(joints_[diff.joint2_index].name + "/position");
            if (!std::isfinite(j1) || !std::isfinite(j2))
            {
                continue;
            }
            double a1 =
                ((j1 - diff.off[0]) * diff.jr[0] + (j2 - diff.off[1]) * diff.jr[1]) *
                diff.ar[0];
            double a2 =
                ((j1 - diff.off[0]) * diff.jr[0] - (j2 - diff.off[1]) * diff.jr[1]) *
                diff.ar[1];
            targets[static_cast<uint8_t>(diff.actuator1_servo_id)] = a1;
            targets[static_cast<uint8_t>(diff.actuator2_servo_id)] = a2;
        }

        // Normalize all targets to [0, 2pi) -- the Dynamixel position range.
        for (auto& [id, pos] : targets)
        {
            (void)id;
            pos = normalizeAnglePositive(pos);
        }

        // Update mocked servo positions (simulates instant tracking) and remove them
        // from the targets map so they are not sent to the physical bus.
        for (auto it = targets.begin(); it != targets.end();)
        {
            if (mock_servo_ids_.count(static_cast<int>(it->first)))
            {
                mock_positions_[it->first] = it->second;
                it = targets.erase(it);
            }
            else
            {
                ++it;
            }
        }

        if (!targets.empty())
        {
            auto result = controller_->setTargetPosition(targets);
            if (!result.isSuccess())
            {
                RCLCPP_WARN(get_logger(), "setTargetPosition failed: %s",
                            dynamixel::getErrorMessage(result.error()).c_str());
            }
        }

        return hardware_interface::return_type::OK;
    }

}  // namespace payloads

PLUGINLIB_EXPORT_CLASS(payloads::DynamixelServos,
                       hardware_interface::SystemInterface)
