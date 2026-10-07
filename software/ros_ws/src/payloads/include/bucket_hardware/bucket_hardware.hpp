// Ensure code is only ran once
#ifndef BUCKET_HARDWARE__BUCKET_HARDWARE_HPP_
#define BUCKET_HARDWARE__BUCKET_HARDWARE_HPP_

#include <array>
#include <hi_can_raw.hpp>
#include <memory>
#include <string>
#include <vector>

#include "bucket_hardware/can_board_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace payloads
{

    /// @brief Per-joint bookkeeping: which CAN actuator (axis + side) this URDF joint
    /// maps to, plus the state/command doubles ros2_control read/writes
    ///
    /// Commands are bank-level (see can_board_interface.hpp), so only ONE joint per axis.
    /// The one declared with command_interfaces in the xacro is commandable
    /// `has_command` reflects this. The other side is state-only: its
    /// command_position field is never written by a controller
    /// and must never be sent to CAN
    struct JointHandle
    {
        std::string name;
        Axis axis;
        Side side;
        bool has_command = false;

        // Firmware frame -> URDF joint frame, from the <ros2_control> joint params:
        //   joint_deg = direction * wrap(firmware_deg - offset_deg)
        double offset_deg = 0.0;
        double direction = 1.0;

        // Command interface (written by a controller, read by write()).
        // Only meaningful when has_command is true.
        double command_position = 0.0;

        // Whether a controller currently has the position command interface
        // claimed. Maintained by perform_command_mode_switch(); write() sends
        // nothing unless this is true, so deactivating the controller really
        // does take this node off the bus.
        bool command_claimed = false;

        // State interfaces (written by read(), read by a controller), in the URDF
        // joint frame: radians and rad/s.
        double state_position = 0.0;
        double state_velocity = 0.0;
        double state_current = 0.0;  // exposed as the "effort" state interface

        // Last position read(), for differentiating state_velocity.
        double previous_position = 0.0;
    };

    /// @brief ros2_control SystemInterface for the bucket's lift/tilt/jaws
    ///
    /// Joints in the URDF's <ros2_control> block (description/ros2_control/
    /// bucket.ros2_control.xacro) are tagged with "axis" (lift|tilt|jaws) and "side"
    /// (left|right), and optionally "offset_deg" and "direction". Exactly one joint
    /// per axis declares a position command interface, mirroring the CAN board only
    /// accepting bank-level commands.
    class BucketHardware : public hardware_interface::SystemInterface
    {
    public:
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo& info) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State& previous_state) override;

        hardware_interface::return_type prepare_command_mode_switch(
            const std::vector<std::string>& start_interfaces,
            const std::vector<std::string>& stop_interfaces) override;

        hardware_interface::return_type perform_command_mode_switch(
            const std::vector<std::string>& start_interfaces,
            const std::vector<std::string>& stop_interfaces) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read(
            const rclcpp::Time& time, const rclcpp::Duration& period) override;

        hardware_interface::return_type write(
            const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        static bool parse_axis(const std::string& value, Axis& out);
        static bool parse_side(const std::string& value, Side& out);

        /// Stops every bank (see CanBoardInterface::stop_axis) and drops every claim.
        void stop_all();

        double to_joint_radians(const JointHandle& joint, double firmware_degrees) const;
        double to_firmware_degrees(const JointHandle& joint, double joint_radians) const;

        /// How far outside its URDF limits a joint may read when a controller claims
        /// it before it is treated as a calibration or direction error rather than
        /// overshoot.
        static constexpr double LIMIT_MARGIN_DEG = 5.0;

        /// Low-pass weight on each new velocity sample; the encoders are 0.1 deg
        /// resolution, so raw differences are steppy.
        static constexpr double VELOCITY_FILTER_ALPHA = 0.3;

        std::vector<JointHandle> joints_;
        std::unique_ptr<CanBoardInterface> can_;

        std::string can_interface_name_ = "can0";  // overridden from <ros2_control> param
    };
}  // namespace payloads

#endif  // BUCKET_HARDWARE__BUCKET_HARDWARE_HPP_
