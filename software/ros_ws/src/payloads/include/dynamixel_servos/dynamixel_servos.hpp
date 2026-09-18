#pragma once

#include <cstdint>
#include <hardware_interface/hardware_component_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <limits>
#include <memory>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace payloads
{
    class DynamixelController;

    /// ros2_control SystemInterface for the Dynamixel bus: Protocol 2.0 multi-turn
    /// position read/write with the Simple and Differential transmissions declared
    /// in arm.ros2_control.xacro applied between joint and actuator space.
    class DynamixelServos : public hardware_interface::SystemInterface
    {
    public:
        DynamixelServos() = default;
        ~DynamixelServos() override;

        hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
            override;

        hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time& time,
                                             const rclcpp::Duration& period) override;

        hardware_interface::return_type
        write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        /// One <joint> of the ros2_control tag, in URDF order.
        struct Joint
        {
            std::string name;
            int servo_id = 0;
            double joint_reduction = 1.0;
            double joint_offset = 0.0;
            double initial = 0.0;
            double lower = -std::numeric_limits<double>::infinity();
            double upper = std::numeric_limits<double>::infinity();
            bool has_transmission = false;
            bool in_differential = false;
            bool is_prismatic = false;

            /// Metres of joint travel per radian of servo rotation. Only meaningful
            /// for prismatic joints driven by a rotary servo (e.g. the gripper).
            double prismatic_scale = 1.0;
        };

        /// A DifferentialTransmission linking two joints to two servo actuators.
        struct DifferentialGroup
        {
            int actuator1_servo_id = 0;
            int actuator2_servo_id = 0;
            double ar[2] = {1.0, 1.0};
            double jr[2] = {1.0, 1.0};
            double off[2] = {0.0, 0.0};
            int joint1_index = 0;
            int joint2_index = 0;
        };

        hardware_interface::CallbackReturn parseJoints();
        hardware_interface::CallbackReturn parseTransmissions();

        /// True if the servo with the given ID is mocked (not on the physical bus).
        bool isMocked(int servo_id) const;

        double forwardTransform(const Joint& joint, double actuator_pos) const;
        double inverseTransform(const Joint& joint, double joint_pos) const;
        std::pair<double, double> differentialJoints(const DifferentialGroup& diff,
                                                     double a1, double a2) const;
        std::pair<double, double> differentialActuators(const DifferentialGroup& diff,
                                                        double j1, double j2) const;

        /// A servo only knows its angle within one turn after power-up. Pick the
        /// turn that keeps each joint inside its limits and nearest its initial
        /// position, and seed mocked servos at that initial position.
        void resolveTurns(const std::unordered_map<uint8_t, int32_t>& counts);
        double toActuator(uint8_t id, int32_t counts) const;
        int32_t toCounts(uint8_t id, double radians) const;
        std::unordered_map<uint8_t, double>
        actuatorPositions(const std::unordered_map<uint8_t, int32_t>& counts) const;

        /// Push actuator positions (rad, keyed by servo ID) through the transmissions
        /// into the joint state interfaces. A zero period reports zero velocity.
        void updateStates(const std::unordered_map<uint8_t, double>& actuators,
                          double period);

        std::unique_ptr<DynamixelController> controller_;
        std::vector<Joint> joints_;
        std::vector<DifferentialGroup> differentials_;
        std::string device_ = "/dev/ttyUSB0";
        int baud_rate_ = 1000000;

        /// Servo IDs to simulate instead of communicating with the physical bus.
        std::set<int> mock_servo_ids_;
        /// Simulated actuator positions for mocked servos (radians).
        std::unordered_map<uint8_t, double> mock_positions_;
        /// Count at actuator zero for each real servo, fixed by resolveTurns().
        std::unordered_map<uint8_t, int32_t> origin_;
    };
}  // namespace payloads
