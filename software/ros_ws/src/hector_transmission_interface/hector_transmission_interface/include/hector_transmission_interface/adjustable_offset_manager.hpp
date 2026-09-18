#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "hector_transmission_interface/adjustable_offset_transmission.hpp"
#include "hector_transmission_interface_msgs/srv/adjust_transmission_offsets.hpp"
#include "hector_transmission_interface_msgs/srv/flip_by_pi.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hector_transmission_interface
{

class IAdjustableOffset
{
public:
  virtual ~IAdjustableOffset() = default;
  virtual void adjustOffset( double offset ) = 0;
  virtual double getOffset() const = 0;
};

class AdjustableOffsetManager
{
public:
  using PositionGetter = std::function<double()>;
  using ServiceCallback = std::function<bool()>;

  struct ManagedJoint {
    std::shared_ptr<IAdjustableOffset> state_handle;
    std::shared_ptr<IAdjustableOffset> command_handle;
    std::shared_ptr<AdjustableOffsetTransmission> state_transmission;
    PositionGetter position_getter;
  };

  explicit AdjustableOffsetManager(
      rclcpp::Node::SharedPtr node,
      std::optional<std::reference_wrapper<std::mutex>> comm_mutex = std::nullopt,
      std::optional<ServiceCallback> pre_callback = std::nullopt,
      std::optional<ServiceCallback> post_callback = std::nullopt );

  /**
   * @brief Add state interface for a joint
   * @param name Joint name
   * @param state_tx State transmission
   * @param position_getter Function to get current position
   */
  void add_joint_state_interface( const std::string &name,
                                  std::shared_ptr<transmission_interface::Transmission> state_tx,
                                  PositionGetter position_getter );

  /**
   * @brief Add command interface for a joint
   * @param name Joint name
   * @param command_tx Command transmission
   */
  void add_joint_command_interface( const std::string &name,
                                    std::shared_ptr<transmission_interface::Transmission> command_tx );

  void add_managed_joint( const std::string &name, ManagedJoint mj );

private:
  /**
   * @brief Wire the 2pi jump correction callback for a joint (if both handles are registered).
   *
   * When both state and command interfaces are registered for a joint, this sets up
   * a callback on the state transmission that automatically syncs the command transmission
   * offset whenever a 2pi jump correction is applied.
   */
  void setupJumpCorrectionCallback( const std::string &name );

  void handle_service(
      const std::shared_ptr<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Request>
          request,
      const std::shared_ptr<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Response>
          response );

  void handle_flip_by_pi(
      const std::shared_ptr<hector_transmission_interface_msgs::srv::FlipByPi::Request> request,
      const std::shared_ptr<hector_transmission_interface_msgs::srv::FlipByPi::Response> response );

  rclcpp::Node::SharedPtr node_;
  std::optional<std::reference_wrapper<std::mutex>> comm_mutex_;
  std::optional<ServiceCallback> pre_callback_;
  std::optional<ServiceCallback> post_callback_;
  std::unordered_map<std::string, ManagedJoint> managed_joints_;
  rclcpp::Service<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets>::SharedPtr service_;
  rclcpp::Service<hector_transmission_interface_msgs::srv::FlipByPi>::SharedPtr flip_by_pi_service_;
};

} // namespace hector_transmission_interface
