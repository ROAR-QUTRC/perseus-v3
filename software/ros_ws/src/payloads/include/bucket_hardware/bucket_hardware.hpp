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
    /// command_position/command_velocity fields are never written by a controller
    /// and must never be sent to CAN
    struct JointHandle
    {
        std::string name;
        Axis axis;
        Side side;
        bool has_command = false;

        // Command interfaces (written by a controller, read by write())
        // Only meaningful when has_command is true
        double command_position = 0.0;
        double command_velocity = 0.0;

        // State interfaces (written by read(), read by a controller)
        double state_position = 0.0;  // Raw angle
        double state_velocity = 0.0;
        double state_current = 0.0;  // exposed as the "effort" state interface

        // ANY OTHER INTERFACES/VALUES go in the above
    };

    /// @brief ros2_control SystemInterface for the bucket's lift/tilt/jaws
    ///
    /// Expects exactly 6 joints in the URDF's <ros2_control> block, each tagges
    /// with "axis" (lift|tilt|jaws) and "side" (left|right) parameters. See
    /// description/bucket.ros2_control.xacro for the expected format.
    /// Exactly one joint per axis must declare command_interfaces (position +
    /// velocity), the other must declare state_interfaces only. This mirrors the CAN
    /// only accepting bank-level commands.
    /// Main software/ros_ws/desrciption/ros2_control.xacro URDF file imports
    /// the bucket.xacro, while maintaining the same names/details for all joints
    /// so they link correctly. The two files have to match, the bucket.xacro is
    /// the local copy used by the ros2_control node.
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

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read(
            const rclcpp::Time& time, const rclcpp::Duration& period) override;

        hardware_interface::return_type write(
            const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        static bool parse_axis(const std::string& value, Axis& out);
        static bool parse_side(const std::string& value, Side& out);

        std::vector<JointHandle> joints_;
        std::unique_ptr<CanBoardInterface> can_;

        std::string can_interface_name_ = "can0";  // overridden from <ros2_control> param
    };
}  // namespace payloads

#endif  // BUCKET_HARDWARE__BUCKET_HARDWARE_HPP_
