#include "bucket_linkage_broadcaster/bucket_linkage_broadcaster.hpp"

#include <algorithm>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "bucket_linkage/bucket_linkage.hpp"

namespace payloads
{

    controller_interface::CallbackReturn BucketLinkageBroadcaster::on_init()
    {
        try
        {
            auto_declare<std::string>("lift_joint", "bucket_lift_joint");
            auto_declare<std::string>("tilt_joint", "bucket_tilt_joint");
            auto_declare<std::string>("jaw_joint", "bucket_jaw_joint");
            // The board only broadcasts every 50 ms, so faster than 20 Hz repeats itself.
            auto_declare<double>("publish_rate", 20.0);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration
    BucketLinkageBroadcaster::command_interface_configuration() const
    {
        return {controller_interface::interface_configuration_type::NONE, {}};
    }

    controller_interface::InterfaceConfiguration
    BucketLinkageBroadcaster::state_interface_configuration() const
    {
        return {controller_interface::interface_configuration_type::INDIVIDUAL,
                {_lift_joint + "/" + hardware_interface::HW_IF_POSITION,
                 _tilt_joint + "/" + hardware_interface::HW_IF_POSITION,
                 _jaw_joint + "/" + hardware_interface::HW_IF_POSITION}};
    }

    controller_interface::CallbackReturn BucketLinkageBroadcaster::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        const auto node = get_node();
        _lift_joint = node->get_parameter("lift_joint").as_string();
        _tilt_joint = node->get_parameter("tilt_joint").as_string();
        _jaw_joint = node->get_parameter("jaw_joint").as_string();
        const double rate = node->get_parameter("publish_rate").as_double();
        _publish_period = rate > 0.0 ? rclcpp::Duration::from_seconds(1.0 / rate)
                                     : rclcpp::Duration(0, 0);

        // Absolute, so it joins the broadcasters' /joint_states that
        // robot_state_publisher reads, whatever namespace the manager runs in.
        _publisher = node->create_publisher<JointState>("/joint_states",
                                                        rclcpp::SystemDefaultsQoS());
        _realtime_publisher =
            std::make_unique<realtime_tools::RealtimePublisher<JointState>>(_publisher);

        _message.name.assign(bucket_linkage::RAM_JOINTS.begin(),
                             bucket_linkage::RAM_JOINTS.end());
        _message.position.assign(bucket_linkage::RAM_JOINTS.size(), 0.0);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BucketLinkageBroadcaster::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        const std::array<std::string, 3> joints{_lift_joint, _tilt_joint, _jaw_joint};
        for (size_t j = 0; j < joints.size(); ++j)
        {
            const auto it = std::find_if(
                state_interfaces_.begin(), state_interfaces_.end(), [&](const auto& interface)
                { return interface.get_prefix_name() == joints[j]; });
            if (it == state_interfaces_.end())
            {
                RCLCPP_ERROR(get_node()->get_logger(), "No position state interface for '%s'",
                             joints[j].c_str());
                return controller_interface::CallbackReturn::ERROR;
            }
            _index[j] = static_cast<size_t>(std::distance(state_interfaces_.begin(), it));
        }
        _last_publish.reset();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type BucketLinkageBroadcaster::update(
        const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
    {
        if (_last_publish && (time - *_last_publish) < _publish_period)
        {
            return controller_interface::return_type::OK;
        }

        std::array<double, 3> q{};
        for (size_t j = 0; j < q.size(); ++j)
        {
            const auto value = state_interfaces_[_index[j]].get_optional();
            if (!value)
            {
                // Hardware busy this cycle; the next one will do.
                return controller_interface::return_type::OK;
            }
            q[j] = *value;
        }

        const auto rams = bucket_linkage::ram_positions(q[0], q[1], q[2]);
        _message.header.stamp = time;
        std::copy(rams.begin(), rams.end(), _message.position.begin());
        if (_realtime_publisher->try_publish(_message))
        {
            _last_publish = time;
        }
        return controller_interface::return_type::OK;
    }

}  // namespace payloads

PLUGINLIB_EXPORT_CLASS(payloads::BucketLinkageBroadcaster, controller_interface::ControllerInterface)
