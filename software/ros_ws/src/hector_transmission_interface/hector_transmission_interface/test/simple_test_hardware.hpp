#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hector_transmission_interface/adjustable_offset_manager.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace hector_transmission_interface
{
namespace test
{

/**
 * @brief A reference implementation of a Hardware Interface using the AdjustableOffsetManager.
 *
 * This class demonstrates how to:
 * 1. Initialize the AdjustableOffsetManager.
 * 2. Configure AdjustableOffsetTransmissions for joints.
 * 3. Register state and command interfaces with the manager.
 * 4. Propagate transmission data in read/write loops.
 */
class SimpleTestHardware : public hardware_interface::SystemInterface
{
public:
  explicit SimpleTestHardware( const std::vector<std::string> &joint_names )
      : joint_names_( joint_names )
  {
  }

  // Convenience constructor for single joint
  explicit SimpleTestHardware( const std::string &joint_name ) : joint_names_( { joint_name } ) { }

  hardware_interface::CallbackReturn on_init( const hardware_interface::HardwareInfo &info ) override
  {
    if ( hardware_interface::SystemInterface::on_init( info ) !=
         hardware_interface::CallbackReturn::SUCCESS ) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // 1. Initialize an internal Node.
    // This node is used by the OffsetManager to provide the adjustment service.
    node_ = std::make_shared<rclcpp::Node>( "simple_test_hw_node" );

    // 2. Initialize the AdjustableOffsetManager.
    // We pass the node and a reference to our communication mutex to ensure thread safety.
    offset_manager_ = std::make_shared<AdjustableOffsetManager>( node_, std::ref( comm_mutex_ ) );

    // 3. Initialize Joint Storage and Transmissions
    for ( const auto &name : joint_names_ ) {
      JointData data;
      data.name = name;
      // Create transmissions. In a real robot, parameters would come from URDF/INFO.
      data.state_transmission = std::make_shared<AdjustableOffsetTransmission>( name, 1.0, 0.0 );
      data.command_transmission = std::make_shared<AdjustableOffsetTransmission>( name, 1.0, 0.0 );
      joints_[name] = data;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
  on_configure( const rclcpp_lifecycle::State & /*previous_state*/ ) override
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
  on_activate( const rclcpp_lifecycle::State & /*previous_state*/ ) override
  {
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

    for ( auto &[name, joint] : joints_ ) {
      // Export the "Joint State" (what ROS sees)
      state_interfaces.emplace_back( std::make_shared<hardware_interface::StateInterface>(
          name, hardware_interface::HW_IF_POSITION, &joint.joint_position ) );

      // Configure Transmission Handles
      std::vector<transmission_interface::JointHandle> joint_handles;
      std::vector<transmission_interface::ActuatorHandle> actuator_handles;

      transmission_interface::JointHandle joint_handle( name, hardware_interface::HW_IF_POSITION,
                                                        &joint.joint_position );
      joint_handles.push_back( joint_handle );

      // In this mock, 'actuator_position' simulates the raw encoder value
      transmission_interface::ActuatorHandle actuator_handle(
          "actuator", hardware_interface::HW_IF_POSITION, &joint.actuator_position );
      actuator_handles.push_back( actuator_handle );

      joint.state_transmission->configure( joint_handles, actuator_handles );

      // 4. Register State Interface with Offset Manager
      // The position_getter lambda provides the current *internal (raw)* position to the manager.
      offset_manager_->add_joint_state_interface( name, joint.state_transmission,
                                                  [&joint]() { return joint.joint_position; } );
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;

    for ( auto &[name, joint] : joints_ ) {
      // Export "Joint Command" (what ROS controllers write to)
      command_interfaces.emplace_back( std::make_shared<hardware_interface::CommandInterface>(
          name, hardware_interface::HW_IF_POSITION, &joint.joint_command ) );

      // Configure Transmission Handles
      std::vector<transmission_interface::JointHandle> joint_handles;
      std::vector<transmission_interface::ActuatorHandle> actuator_handles;

      transmission_interface::JointHandle joint_handle( name, hardware_interface::HW_IF_POSITION,
                                                        &joint.joint_command );
      joint_handles.push_back( joint_handle );

      // Actuator command buffer
      transmission_interface::ActuatorHandle actuator_handle(
          "actuator", hardware_interface::HW_IF_POSITION, &joint.actuator_command );
      actuator_handles.push_back( actuator_handle );

      joint.command_transmission->configure( joint_handles, actuator_handles );

      // 5. Register Command Interface with Offset Manager
      offset_manager_->add_joint_command_interface( name, joint.command_transmission );
    }

    return command_interfaces;
  }

  hardware_interface::return_type read( const rclcpp::Time & /*time*/,
                                        const rclcpp::Duration & /*period*/ ) override
  {
    std::lock_guard<std::mutex> lock( comm_mutex_ );
    // In a real HW, we would read from device to 'actuator_position' here.
    // Then we apply transmission:
    for ( auto &[name, joint] : joints_ ) {
      if ( joint.state_transmission ) {
        joint.state_transmission->actuator_to_joint();
      }
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write( const rclcpp::Time & /*time*/,
                                         const rclcpp::Duration & /*period*/ ) override
  {
    std::lock_guard<std::mutex> lock( comm_mutex_ );
    for ( auto &[name, joint] : joints_ ) {
      if ( joint.command_transmission ) {
        joint.command_transmission->joint_to_actuator();
      }
    }
    // In a real HW, we would write 'actuator_command' to device here.
    return hardware_interface::return_type::OK;
  }

  // --- Helpers for Testing ---

  rclcpp::Node::SharedPtr get_node() { return node_; }

  /**
   * @brief Set the raw actuator position for test simulation.
   */
  void set_raw_actuator_position( const std::string &joint_name, double val )
  {
    if ( joints_.count( joint_name ) ) {
      joints_[joint_name].actuator_position = val;
    }
  }

  /**
   * @brief Get the current high-level joint position.
   * This is the value ROS sees (Raw + Offset).
   */
  double get_joint_position( const std::string &joint_name )
  {
    if ( joints_.count( joint_name ) ) {
      return joints_[joint_name].joint_position;
    }
    return std::numeric_limits<double>::quiet_NaN();
  }

  /**
   * @brief Manually trigger the read cycle logic for testing.
   */
  void propagate_read()
  {
    std::lock_guard<std::mutex> lock( comm_mutex_ );

    for ( auto &[name, joint] : joints_ ) {
      if ( joint.state_transmission ) {
        joint.state_transmission->actuator_to_joint();
      }
    }
  }

private:
public: // Made public struct for simplicity in this single-header test file
  struct JointData {
    std::string name;
    double joint_position = 0.0;
    double joint_command = 0.0;
    double actuator_position = 0.0;
    double actuator_command = 0.0;
    std::shared_ptr<AdjustableOffsetTransmission> state_transmission;
    std::shared_ptr<AdjustableOffsetTransmission> command_transmission;
  };

private:
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, JointData> joints_;

  rclcpp::Node::SharedPtr node_;
  std::mutex comm_mutex_;
  std::shared_ptr<AdjustableOffsetManager> offset_manager_;
};

} // namespace test
} // namespace hector_transmission_interface
