#include "dynamixel_servos/dynamixel_servos.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <pluginlib/class_list_macros.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "dynamixel_servos/dynamixel_controller.hpp"

namespace payloads
{
    namespace
    {
        constexpr double tau = 6.28318530717958647692;
        constexpr int32_t counts_per_turn = DynamixelController::counts_per_turn;
        /// Servo turns searched either side of the raw reading for a geared joint.
        constexpr int turn_window = 16;
    }  // namespace

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

            for (const auto& state : joint_info.state_interfaces)
            {
                if (state.name == "position" && !state.initial_value.empty())
                {
                    j.initial = std::atof(state.initial_value.c_str());
                }
            }
            auto limits = info_.limits.find(j.name);
            if (limits != info_.limits.end() && limits->second.has_position_limits)
            {
                j.lower = limits->second.min_position;
                j.upper = limits->second.max_position;
            }

            joints_.push_back(std::move(j));
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn DynamixelServos::parseTransmissions()
    {
        for (const auto& trans : info_.transmissions)
        {
            if (trans.type == "transmission_interface/SimpleTransmission" ||
                trans.type == "hector_transmission_interface/AdjustableOffsetTransmission")
            {
                if (trans.joints.size() != 1)
                {
                    RCLCPP_ERROR(get_logger(),
                                 "Transmission '%s' must have exactly one joint",
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
                                 "Transmission '%s' references unknown joint '%s'",
                                 trans.name.c_str(), j.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }
                it->has_transmission = true;
                double actuator_reduction =
                    (!trans.actuators.empty() && trans.actuators[0].mechanical_reduction != 0.0)
                        ? trans.actuators[0].mechanical_reduction
                        : 1.0;
                it->joint_reduction = j.mechanical_reduction * actuator_reduction;
                it->joint_offset = j.offset;

                if (trans.type == "hector_transmission_interface/AdjustableOffsetTransmission")
                {
                    const char* home = std::getenv("HOME");
                    if (home != nullptr)
                    {
                        std::filesystem::path offset_file = std::filesystem::path(home) / ".ros" /
                                                            "dynamic_offset_transmissions" / (j.name + ".txt");
                        if (std::filesystem::exists(offset_file))
                        {
                            std::ifstream file(offset_file);
                            double saved_offset = 0.0;
                            if (file >> saved_offset)
                            {
                                it->joint_offset = saved_offset;
                                RCLCPP_INFO(get_logger(),
                                            "Loaded dynamic offset for %s: %.4f from %s",
                                            j.name.c_str(), saved_offset,
                                            offset_file.string().c_str());
                            }
                        }
                    }
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
        double joint_pos = joint.has_transmission
                               ? actuator_pos / joint.joint_reduction + joint.joint_offset
                               : actuator_pos;
        return joint.is_prismatic ? joint_pos * joint.prismatic_scale : joint_pos;
    }

    double DynamixelServos::inverseTransform(const Joint& joint, double joint_pos) const
    {
        if (joint.is_prismatic)
        {
            joint_pos /= joint.prismatic_scale;
        }
        return joint.has_transmission
                   ? (joint_pos - joint.joint_offset) * joint.joint_reduction
                   : joint_pos;
    }

    std::pair<double, double>
    DynamixelServos::differentialJoints(const DifferentialGroup& diff, double a1,
                                        double a2) const
    {
        double sum = a1 / diff.ar[0] + a2 / diff.ar[1];
        double difference = a1 / diff.ar[0] - a2 / diff.ar[1];
        return {sum / (2.0 * diff.jr[0]) + diff.off[0],
                difference / (2.0 * diff.jr[1]) + diff.off[1]};
    }

    std::pair<double, double>
    DynamixelServos::differentialActuators(const DifferentialGroup& diff, double j1,
                                           double j2) const
    {
        double p = (j1 - diff.off[0]) * diff.jr[0];
        double q = (j2 - diff.off[1]) * diff.jr[1];
        return {(p + q) * diff.ar[0], (p - q) * diff.ar[1]};
    }

    double DynamixelServos::toActuator(uint8_t id, int32_t counts) const
    {
        return (counts - origin_.at(id)) * tau / counts_per_turn;
    }

    int32_t DynamixelServos::toCounts(uint8_t id, double radians) const
    {
        long long counts = std::llround(radians * counts_per_turn / tau) + origin_.at(id);
        return static_cast<int32_t>(std::clamp<long long>(
            counts, -DynamixelController::count_limit, DynamixelController::count_limit));
    }

    void DynamixelServos::resolveTurns(const std::unordered_map<uint8_t, int32_t>& counts)
    {
        std::set<uint8_t> restored;
        auto candidates = [&](int servo_id, double image)
        {
            std::vector<double> out;
            if (isMocked(servo_id))
            {
                out.push_back(image);
                return out;
            }
            auto id = static_cast<uint8_t>(servo_id);
            double raw = (counts.at(id) - DynamixelController::center) * tau / counts_per_turn;
            auto saved = controller_->savedTurn(id);
            if (saved.isSuccess() && saved.value())
            {
                restored.insert(id);
                out.push_back(raw + *saved.value() * tau);
                return out;
            }
            for (int k = -turn_window; k <= turn_window; ++k)
            {
                out.push_back(raw + k * tau);
            }
            return out;
        };
        // Any in-limit candidate beats every out-of-limit one; ties go to initial.
        auto score = [](const Joint& joint, double pos)
        {
            double delta = pos - joint.initial;
            return (pos < joint.lower || pos > joint.upper ? 1e6 : 0.0) + delta * delta;
        };
        auto commit = [&](const Joint& joint, double actuator)
        {
            auto id = static_cast<uint8_t>(joint.servo_id);
            if (isMocked(joint.servo_id))
            {
                mock_positions_[id] = actuator;
                return;
            }
            origin_[id] = counts.at(id) - static_cast<int32_t>(std::llround(
                                              actuator * counts_per_turn / tau));
            int turn = (DynamixelController::center - origin_[id]) / counts_per_turn;
            (void)controller_->saveTurn(id, turn);
            RCLCPP_INFO(get_logger(), "Servo %d (%s): turn %+d %s, position %.3f",
                        joint.servo_id, joint.name.c_str(), turn,
                        restored.count(id) ? "restored" : "resolved",
                        forwardTransform(joint, actuator));
        };

        for (const auto& joint : joints_)
        {
            if (joint.in_differential)
            {
                continue;
            }
            double best = 0.0;
            double best_score = std::numeric_limits<double>::infinity();
            for (double a : candidates(joint.servo_id, inverseTransform(joint, joint.initial)))
            {
                double s = score(joint, forwardTransform(joint, a));
                if (s < best_score)
                {
                    best_score = s;
                    best = a;
                }
            }
            commit(joint, best);
        }

        for (const auto& diff : differentials_)
        {
            const Joint& j1 = joints_[diff.joint1_index];
            const Joint& j2 = joints_[diff.joint2_index];
            auto [i1, i2] = differentialActuators(diff, j1.initial, j2.initial);
            double best1 = 0.0;
            double best2 = 0.0;
            double best_score = std::numeric_limits<double>::infinity();
            for (double a1 : candidates(diff.actuator1_servo_id, i1))
            {
                for (double a2 : candidates(diff.actuator2_servo_id, i2))
                {
                    auto [p1, p2] = differentialJoints(diff, a1, a2);
                    double s = score(j1, p1) + score(j2, p2);
                    if (s < best_score)
                    {
                        best_score = s;
                        best1 = a1;
                        best2 = a2;
                    }
                }
            }
            commit(j1, best1);
            commit(j2, best2);
        }
    }

    std::unordered_map<uint8_t, double> DynamixelServos::actuatorPositions(
        const std::unordered_map<uint8_t, int32_t>& counts) const
    {
        auto positions = mock_positions_;
        for (const auto& [id, count] : counts)
        {
            if (!isMocked(id))
            {
                positions[id] = toActuator(id, count);
            }
        }
        return positions;
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
            auto id = static_cast<uint8_t>(joint.servo_id);
            if (auto result = controller_->configure(id); !result.isSuccess())
            {
                RCLCPP_ERROR(get_logger(), "Failed to configure servo %d: %s",
                             joint.servo_id,
                             dynamixel::getErrorMessage(result.error()).c_str());
                controller_.reset();
                return hardware_interface::CallbackReturn::FAILURE;
            }
        }

        auto counts = controller_->readCounts();
        if (!counts.isSuccess())
        {
            RCLCPP_ERROR(get_logger(), "Initial position read failed: %s",
                         dynamixel::getErrorMessage(counts.error()).c_str());
            controller_.reset();
            return hardware_interface::CallbackReturn::FAILURE;
        }
        resolveTurns(counts.value());
        updateStates(actuatorPositions(counts.value()), 0.0);
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

    void DynamixelServos::updateStates(
        const std::unordered_map<uint8_t, double>& actuators, double period)
    {
        auto set = [&](const std::string& name, double position)
        {
            double velocity =
                period > 0.0 ? (position - get_state(name + "/position")) / period : 0.0;
            set_state(name + "/position", position);
            set_state(name + "/velocity", velocity);
        };

        for (const auto& diff : differentials_)
        {
            auto a1 = actuators.find(static_cast<uint8_t>(diff.actuator1_servo_id));
            auto a2 = actuators.find(static_cast<uint8_t>(diff.actuator2_servo_id));
            if (a1 == actuators.end() || a2 == actuators.end())
            {
                continue;
            }
            auto [j1, j2] = differentialJoints(diff, a1->second, a2->second);
            set(joints_[diff.joint1_index].name, j1);
            set(joints_[diff.joint2_index].name, j2);
        }

        for (const auto& joint : joints_)
        {
            if (joint.in_differential)
            {
                continue;
            }
            auto it = actuators.find(static_cast<uint8_t>(joint.servo_id));
            if (it != actuators.end())
            {
                set(joint.name, forwardTransform(joint, it->second));
            }
        }
    }

    hardware_interface::return_type
    DynamixelServos::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
    {
        auto counts = controller_->readCounts();
        if (!counts.isSuccess())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "readCounts failed: %s",
                                 dynamixel::getErrorMessage(counts.error()).c_str());
            return hardware_interface::return_type::OK;
        }
        updateStates(actuatorPositions(counts.value()), period.seconds());
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type
    DynamixelServos::write(const rclcpp::Time& /*time*/,
                           const rclcpp::Duration& /*period*/)
    {
        std::unordered_map<uint8_t, double> targets;
        // Never clamp harder than where the joint already is, so one found outside
        // its limits is not yanked to the limit at full speed.
        auto bounded = [&](const Joint& joint, double cmd)
        {
            double current = get_state(joint.name + "/position");
            return std::clamp(cmd, std::min(joint.lower, current),
                              std::max(joint.upper, current));
        };

        for (const auto& joint : joints_)
        {
            if (joint.in_differential)
            {
                continue;
            }
            double cmd = get_command(joint.name + "/position");
            // Controllers have not written a command yet on the first few cycles.
            if (!std::isfinite(cmd))
            {
                continue;
            }
            targets[static_cast<uint8_t>(joint.servo_id)] =
                inverseTransform(joint, bounded(joint, cmd));
        }

        for (const auto& diff : differentials_)
        {
            const Joint& j1 = joints_[diff.joint1_index];
            const Joint& j2 = joints_[diff.joint2_index];
            double c1 = get_command(j1.name + "/position");
            double c2 = get_command(j2.name + "/position");
            if (!std::isfinite(c1) || !std::isfinite(c2))
            {
                continue;
            }
            auto [a1, a2] = differentialActuators(diff, bounded(j1, c1), bounded(j2, c2));
            targets[static_cast<uint8_t>(diff.actuator1_servo_id)] = a1;
            targets[static_cast<uint8_t>(diff.actuator2_servo_id)] = a2;
        }

        // Mocked servos track their target instantly and never reach the bus.
        std::unordered_map<uint8_t, int32_t> counts;
        for (const auto& [id, actuator] : targets)
        {
            if (isMocked(id))
            {
                mock_positions_[id] = actuator;
            }
            else
            {
                counts[id] = toCounts(id, actuator);
            }
        }

        if (auto result = controller_->setGoalCounts(counts); !result.isSuccess())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "setGoalCounts failed: %s",
                                 dynamixel::getErrorMessage(result.error()).c_str());
        }

        return hardware_interface::return_type::OK;
    }

}  // namespace payloads

PLUGINLIB_EXPORT_CLASS(payloads::DynamixelServos,
                       hardware_interface::SystemInterface)
