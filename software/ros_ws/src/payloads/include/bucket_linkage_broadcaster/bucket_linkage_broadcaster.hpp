#pragma once

#include <array>
#include <controller_interface/controller_interface.hpp>
#include <memory>
#include <optional>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

namespace payloads
{

    /// @brief Publishes the bucket's ten ram joints, solved from lift, tilt and jaw.
    ///
    /// The rams close kinematic loops, so they are not independent joints and no
    /// hardware reports them, but robot_state_publisher needs a value for each to
    /// draw the bucket. This reads the three driven joints' position state
    /// interfaces every update and publishes the rams on /joint_states, beside
    /// bucket_joint_state_broadcaster's lift/tilt/jaw, with the same stamp - so the
    /// whole linkage moves together straight from the encoder stream, real or mock.
    ///
    /// Read-only: it claims no command interfaces. The kinematics are
    /// bucket_linkage.hpp.
    class BucketLinkageBroadcaster : public controller_interface::ControllerInterface
    {
    public:
        controller_interface::CallbackReturn on_init() override;
        controller_interface::InterfaceConfiguration command_interface_configuration()
            const override;
        controller_interface::InterfaceConfiguration state_interface_configuration()
            const override;
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::return_type update(const rclcpp::Time& time,
                                                 const rclcpp::Duration& period) override;

    private:
        using JointState = sensor_msgs::msg::JointState;

        std::string _lift_joint;
        std::string _tilt_joint;
        std::string _jaw_joint;

        /// Indices into state_interfaces_ for lift, tilt and jaw, resolved by name on
        /// activation rather than assuming the manager kept the configured order.
        std::array<size_t, 3> _index{};

        /// Minimum time between publishes; zero publishes every update.
        rclcpp::Duration _publish_period{0, 0};
        std::optional<rclcpp::Time> _last_publish;

        rclcpp::Publisher<JointState>::SharedPtr _publisher;
        std::unique_ptr<realtime_tools::RealtimePublisher<JointState>> _realtime_publisher;
        JointState _message;
    };

}  // namespace payloads
