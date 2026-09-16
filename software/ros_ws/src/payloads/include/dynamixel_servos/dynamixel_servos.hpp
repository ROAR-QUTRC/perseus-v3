#pragma once

#include <cstdint>
#include <hardware_interface/hardware_component_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <memory>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace payloads
{
    class DynamixelController;

    /// ros2_control SystemInterface plugin for the Dynamixel servo bus.
    ///
    /// Reads present positions and writes goal positions over Protocol 2.0,
    /// applying the SimpleTransmission and DifferentialTransmission entries
    /// declared in arm.ros2_control.xacro to convert between joint space and
    /// actuator space.
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
            double actuator_reduction = 1.0;
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

        /// Apply forward transmission (actuator -> joint) for a single joint.
        double forwardTransform(const Joint& joint, double actuator_pos) const;

        /// Apply inverse transmission (joint -> actuator) for a single joint.
        double inverseTransform(const Joint& joint, double joint_pos) const;

        /// Wrap an angle to [-pi, pi].
        static double normalizeAngle(double angle);

        /// Wrap an angle to [0, 2pi).
        static double normalizeAnglePositive(double angle);

        std::unique_ptr<DynamixelController> controller_;
        std::vector<Joint> joints_;
        std::vector<DifferentialGroup> differentials_;
        std::string device_ = "/dev/ttyUSB0";
        int baud_rate_ = 1000000;

        /// Servo IDs to simulate instead of communicating with the physical bus.
        std::set<int> mock_servo_ids_;
        /// Simulated actuator positions for mocked servos (radians).
        std::unordered_map<uint8_t, double> mock_positions_;
    };
}  // namespace payloads
