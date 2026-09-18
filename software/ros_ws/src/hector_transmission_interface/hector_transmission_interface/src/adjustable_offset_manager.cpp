#include "hector_transmission_interface/adjustable_offset_manager.hpp"

#include <cmath>

namespace hector_transmission_interface
{
/**
 * @brief Concrete adapter mapping the interface to the non-virtual transmission methods.
 */
class AdjustableTransmissionAdapter : public IAdjustableOffset
{
public:
  explicit AdjustableTransmissionAdapter( std::shared_ptr<AdjustableOffsetTransmission> tx )
      : tx_( tx )
  {
  }

  void adjustOffset( double offset ) override { tx_->adjustTransmissionOffset( offset ); }
  double getOffset() const override { return tx_->get_joint_offset(); }

private:
  std::shared_ptr<AdjustableOffsetTransmission> tx_;
};

AdjustableOffsetManager::AdjustableOffsetManager(
    rclcpp::Node::SharedPtr node, std::optional<std::reference_wrapper<std::mutex>> comm_mutex,
    std::optional<ServiceCallback> pre_callback, std::optional<ServiceCallback> post_callback )
    : node_( node ), comm_mutex_( comm_mutex ), pre_callback_( pre_callback ),
      post_callback_( post_callback )
{
  service_ = node_->create_service<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets>(
      "~/adjust_transmission_offsets", std::bind( &AdjustableOffsetManager::handle_service, this,
                                                  std::placeholders::_1, std::placeholders::_2 ) );
  flip_by_pi_service_ = node_->create_service<hector_transmission_interface_msgs::srv::FlipByPi>(
      "~/flip_by_pi", std::bind( &AdjustableOffsetManager::handle_flip_by_pi, this,
                                 std::placeholders::_1, std::placeholders::_2 ) );
}

void AdjustableOffsetManager::add_joint_state_interface(
    const std::string &name, std::shared_ptr<transmission_interface::Transmission> state_tx,
    PositionGetter position_getter )
{
  auto adj_state = std::dynamic_pointer_cast<AdjustableOffsetTransmission>( state_tx );

  if ( adj_state ) {
    auto it = managed_joints_.find( name );
    if ( it == managed_joints_.end() ) {
      ManagedJoint mj;
      mj.state_handle = std::make_shared<AdjustableTransmissionAdapter>( adj_state );
      mj.state_transmission = adj_state;
      mj.position_getter = position_getter;
      managed_joints_[name] = mj;
    } else {
      it->second.state_handle = std::make_shared<AdjustableTransmissionAdapter>( adj_state );
      it->second.state_transmission = adj_state;
      it->second.position_getter = position_getter;
    }
    RCLCPP_INFO( node_->get_logger(), "Registered state interface for joint '%s'.", name.c_str() );
    setupJumpCorrectionCallback( name );
  }
}

void AdjustableOffsetManager::add_joint_command_interface(
    const std::string &name, std::shared_ptr<transmission_interface::Transmission> command_tx )
{
  auto adj_cmd = std::dynamic_pointer_cast<AdjustableOffsetTransmission>( command_tx );

  if ( adj_cmd ) {
    auto it = managed_joints_.find( name );
    if ( it == managed_joints_.end() ) {
      ManagedJoint mj;
      mj.command_handle = std::make_shared<AdjustableTransmissionAdapter>( adj_cmd );
      managed_joints_[name] = mj;
    } else {
      it->second.command_handle = std::make_shared<AdjustableTransmissionAdapter>( adj_cmd );
    }
    RCLCPP_INFO( node_->get_logger(), "Registered command interface for joint '%s'.", name.c_str() );
    setupJumpCorrectionCallback( name );
  }
}

void AdjustableOffsetManager::add_managed_joint( const std::string &name, ManagedJoint mj )
{
  managed_joints_[name] = mj;
}

void AdjustableOffsetManager::setupJumpCorrectionCallback( const std::string &name )
{
  auto it = managed_joints_.find( name );
  if ( it == managed_joints_.end() )
    return;

  auto &mj = it->second;

  // Only wire the callback once both state transmission and command handle are available
  if ( !mj.state_transmission || !mj.command_handle )
    return;

  auto command_handle = mj.command_handle;
  auto logger = node_->get_logger();
  std::string joint_name = name;

  mj.state_transmission->setOffsetCorrectionCallback(
      [command_handle, logger, joint_name]( double new_offset ) {
        command_handle->adjustOffset( new_offset );
        RCLCPP_WARN( logger,
                     "Synced command transmission offset for joint '%s' to %.6f "
                     "after automatic actuator jump correction.",
                     joint_name.c_str(), new_offset );
      } );
}

void AdjustableOffsetManager::handle_service(
    const std::shared_ptr<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Request> request,
    const std::shared_ptr<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Response>
        response )
{
  // --- PRE CALLBACK ---
  if ( pre_callback_.has_value() ) {
    if ( !pre_callback_.value()() ) {
      RCLCPP_WARN( node_->get_logger(), "Pre-callback returned false, aborting service call." );
      response->success = false;
      response->message = "Pre-callback failed";
      return;
    }
  }

  // --- THREAD SAFETY ---
  std::unique_lock<std::mutex> lock;
  if ( comm_mutex_.has_value() ) {
    lock = std::unique_lock<std::mutex>( comm_mutex_->get() );
    RCLCPP_DEBUG( node_->get_logger(), "Offset adjustment service locked hardware mutex." );
  }

  response->success = false;

  size_t num_measurements = std::min( request->external_joint_measurements.name.size(),
                                      request->external_joint_measurements.position.size() );

  for ( size_t i = 0; i < num_measurements; ++i ) {
    const std::string &joint_name = request->external_joint_measurements.name[i];
    double external_pos = request->external_joint_measurements.position[i];

    if ( !std::isfinite( external_pos ) ) {
      RCLCPP_WARN( node_->get_logger(), "Received NaN for joint %s, skipping.", joint_name.c_str() );
      continue;
    }

    auto it = managed_joints_.find( joint_name );
    if ( it == managed_joints_.end() )
      continue;

    auto &mj = it->second;

    // Check that both handles are registered
    if ( !mj.state_handle || !mj.command_handle ) {
      RCLCPP_WARN(
          node_->get_logger(),
          "Joint '%s' does not have both state and command interfaces registered, skipping.",
          joint_name.c_str() );
      continue;
    }

    // check that state and command interface are not the same
    if ( mj.state_handle == mj.command_handle ) {
      RCLCPP_WARN( node_->get_logger(),
                   "Joint '%s' has the same transmission for state and command interface "
                   "registered in the AdjustableOffsetManager, skipping.",
                   joint_name.c_str() );
      continue;
    }

    double internal_pos = 0.0;
    bool found_internal = false;

    if ( !request->original_joint_state.name.empty() ) {
      for ( size_t j = 0; j < request->original_joint_state.name.size(); ++j ) {
        if ( request->original_joint_state.name[j] == joint_name ) {
          internal_pos = request->original_joint_state.position[j];
          found_internal = std::isfinite( internal_pos );
          break;
        }
      }
    }

    if ( !found_internal )
      internal_pos = mj.position_getter();

    double current_offset = mj.state_handle->getOffset();
    double corrected_offset =
        request->external_joint_measurements.position[i] - internal_pos + current_offset;

    mj.state_handle->adjustOffset( corrected_offset );
    mj.command_handle->adjustOffset( corrected_offset );

    response->offset_changes.push_back( corrected_offset - current_offset );
  }
  response->message = response->offset_changes.empty() ? "No joints adjusted" : "Success";
  response->success = !response->offset_changes.empty();

  // --- POST CALLBACK ---
  if ( post_callback_.has_value() ) {
    if ( !post_callback_.value()() ) {
      RCLCPP_WARN( node_->get_logger(), "Post-callback returned false." );
    }
  }
}
void AdjustableOffsetManager::handle_flip_by_pi(
    const std::shared_ptr<hector_transmission_interface_msgs::srv::FlipByPi::Request> request,
    const std::shared_ptr<hector_transmission_interface_msgs::srv::FlipByPi::Response> response )
{
  // --- PRE CALLBACK ---
  if ( pre_callback_.has_value() ) {
    if ( !pre_callback_.value()() ) {
      RCLCPP_WARN( node_->get_logger(),
                   "Pre-callback returned false, aborting flip_by_pi service call." );
      response->success = false;
      response->message = "Pre-callback failed";
      return;
    }
  }

  // --- THREAD SAFETY ---
  std::unique_lock<std::mutex> lock;
  if ( comm_mutex_.has_value() ) {
    lock = std::unique_lock<std::mutex>( comm_mutex_->get() );
    RCLCPP_DEBUG( node_->get_logger(), "flip_by_pi service locked hardware mutex." );
  }

  size_t flipped_count = 0;

  for ( const auto &joint_name : request->joint_names ) {
    auto it = managed_joints_.find( joint_name );
    if ( it == managed_joints_.end() ) {
      RCLCPP_WARN( node_->get_logger(), "Joint '%s' not found, skipping.", joint_name.c_str() );
      continue;
    }

    auto &mj = it->second;

    if ( !mj.state_handle || !mj.command_handle ) {
      RCLCPP_WARN(
          node_->get_logger(),
          "Joint '%s' does not have both state and command interfaces registered, skipping.",
          joint_name.c_str() );
      continue;
    }

    double new_state_offset = mj.state_handle->getOffset() + M_PI;
    double new_command_offset = mj.command_handle->getOffset() + M_PI;

    mj.state_handle->adjustOffset( new_state_offset );
    mj.command_handle->adjustOffset( new_command_offset );

    RCLCPP_INFO( node_->get_logger(), "Flipped joint '%s' by pi.", joint_name.c_str() );
    ++flipped_count;
  }

  response->success = flipped_count > 0;
  response->message = flipped_count > 0
                          ? "Flipped " + std::to_string( flipped_count ) + " joint(s) by pi"
                          : "No joints flipped";

  // --- POST CALLBACK ---
  if ( post_callback_.has_value() ) {
    if ( !post_callback_.value()() ) {
      RCLCPP_WARN( node_->get_logger(), "Post-callback returned false." );
    }
  }
}

} // namespace hector_transmission_interface
