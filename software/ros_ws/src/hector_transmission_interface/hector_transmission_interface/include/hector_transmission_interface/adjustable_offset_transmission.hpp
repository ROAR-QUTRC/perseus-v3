//
// Created by aljoscha-schmidt on 6/4/25.
//

#ifndef DYNAMIC_OFFSET_TRANSMISSION_HPP
#define DYNAMIC_OFFSET_TRANSMISSION_HPP

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <transmission_interface/simple_transmission.hpp>
#include <utility>
#include <vector>

namespace hector_transmission_interface
{
class AdjustableOffsetTransmission : public transmission_interface::SimpleTransmission
{
public:
  using OffsetCorrectionCallback = std::function<void( double new_offset )>;

  explicit AdjustableOffsetTransmission( const std::string &joint_name,
                                         double joint_to_actuator_reduction,
                                         double joint_offset = 0.0, bool pass_through_effort = false,
                                         std::vector<std::string> pass_through_interfaces = {} );
  /***
   * \brief
   * Adjusts the transmission offset for the joint.
   * This will also save the new offset to the file system.
   * WARNING: MAKE SURE TO DEACTIVATE THE CONTROLLER BEFORE CHANGING THE OFFSET!
   * @param offset The new offset to set for the joint.
   */
  void adjustTransmissionOffset( double offset );

  /**
   * \brief Override configure to optionally route effort/torque/force and arbitrary
   * named interfaces through identity passthrough instead of letting the base class
   * scale them by the reduction.
   *
   * Handles whose interface name is in the passthrough set are removed from the lists
   * passed to SimpleTransmission::configure (so the base scaling never sees them) and
   * stored in passthrough_pairs_ for direct identity copy in actuator_to_joint /
   * joint_to_actuator.
   */
  void configure( const std::vector<transmission_interface::JointHandle> &joint_handles,
                  const std::vector<transmission_interface::ActuatorHandle> &actuator_handles ) override;

  /**
   * \brief Override actuator_to_joint to detect and correct 2pi jumps on the actuator side.
   *
   * When a Dynamixel actuator briefly loses power, it can lose its absolute position
   * and reset by a multiple of 2pi. On the joint side (with a transmission ratio),
   * this manifests as a jump of 2pi/reduction. This method detects such jumps
   * and compensates by adjusting the joint offset.
   *
   * Also identity-copies any passthrough interfaces from actuator to joint side.
   */
  void actuator_to_joint() override;

  /**
   * \brief Override joint_to_actuator to identity-copy any passthrough interfaces
   * from joint to actuator side after the base scaling.
   */
  void joint_to_actuator() override;

  /**
   * \brief Returns the number of 2pi corrections applied since construction (for diagnostics).
   */
  int getCorrectionCount() const { return correction_count_; }

  /**
   * \brief Register a callback that is invoked when a 2pi jump correction is applied.
   *
   * The callback receives the corrected offset value. This is used by the
   * AdjustableOffsetManager to automatically synchronize the command transmission.
   */
  void setOffsetCorrectionCallback( OffsetCorrectionCallback callback )
  {
    offset_correction_callback_ = std::move( callback );
  }

private:
  bool existsTransmissionOffsetFile() const;
  void loadTransmissionOffset();
  void saveTransmissionOffset() const;

  std::string joint_name_;
  std::filesystem::path transmission_file_path_;

  // 2pi jump detection state
  std::optional<double> prev_actuator_position_;
  int correction_count_{ 0 };
  OffsetCorrectionCallback offset_correction_callback_;

  // Passthrough configuration
  bool pass_through_effort_;
  std::vector<std::string> pass_through_interface_names_;
  // Pairs of (joint_handle, actuator_handle) sharing the same interface name that
  // should be identity-copied in both directions instead of being scaled by reduction.
  std::vector<std::pair<transmission_interface::JointHandle, transmission_interface::ActuatorHandle>>
      passthrough_pairs_;

  static constexpr double TWO_PI = 2.0 * M_PI;
  // Tolerance for detecting a 2pi jump (allow ~10% margin)
  static constexpr double JUMP_TOLERANCE = 0.2 * M_PI;
};

inline AdjustableOffsetTransmission::AdjustableOffsetTransmission(
    const std::string &joint_name, double joint_to_actuator_reduction, double joint_offset,
    bool pass_through_effort, std::vector<std::string> pass_through_interfaces )
    : SimpleTransmission( joint_to_actuator_reduction, joint_offset ), joint_name_( joint_name ),
      pass_through_effort_( pass_through_effort ),
      pass_through_interface_names_( std::move( pass_through_interfaces ) )
{
  // path is ~/.ros/dynamic_offset_transmissions/ + joint_name + ".yaml"
  transmission_file_path_ = std::filesystem::path( std::getenv( "HOME" ) ) / ".ros" /
                            "dynamic_offset_transmissions" / ( joint_name + ".txt" );
  const auto old_offset = get_joint_offset();
  if ( existsTransmissionOffsetFile() ) {
    loadTransmissionOffset();
  } else {
    saveTransmissionOffset(); // save the initial offset to file
  }
  if ( old_offset != get_joint_offset() ) {
    RCLCPP_INFO( rclcpp::get_logger( "AdjustableOffsetTransmission" ),
                 "Loaded transmission offset for joint '%s': %f from file %s", joint_name_.c_str(),
                 get_joint_offset(), transmission_file_path_.c_str() );
  }
}

inline bool AdjustableOffsetTransmission::existsTransmissionOffsetFile() const
{
  return std::filesystem::exists( transmission_file_path_ );
}

inline void AdjustableOffsetTransmission::loadTransmissionOffset()
{
  // Load the transmission offset from the specified storage path
  if ( std::filesystem::exists( transmission_file_path_ ) ) {
    std::ifstream file( transmission_file_path_ );
    if ( file.is_open() ) {
      double offset;
      file >> offset;
      if ( file.fail() || !file.eof() ) {
        RCLCPP_ERROR( rclcpp::get_logger( "AdjustableOffsetTransmission" ),
                      "Failed to read a valid offset from file: %s",
                      transmission_file_path_.string().c_str() );
      } else {
        jnt_offset_ = offset;
      }
      file.close();
    } else {
      RCLCPP_ERROR( rclcpp::get_logger( "AdjustableOffsetTransmission" ),
                    "Failed to open transmission offset file: %s",
                    transmission_file_path_.string().c_str() );
    }
  } else {
    RCLCPP_WARN( rclcpp::get_logger( "AdjustableOffsetTransmission" ),
                 "Transmission offset file does not exist: %s",
                 transmission_file_path_.string().c_str() );
  }
}

inline void AdjustableOffsetTransmission::saveTransmissionOffset() const
{
  // Save the current transmission offset to the specified storage path
  std::filesystem::create_directories( transmission_file_path_.parent_path() );
  std::ofstream file( transmission_file_path_ );
  if ( file.is_open() ) {
    file << jnt_offset_;
    if ( file.fail() ) {
      RCLCPP_ERROR( rclcpp::get_logger( "AdjustableOffsetTransmission" ),
                    "Failed to write offset to file: %s", transmission_file_path_.string().c_str() );
    }
    file.close();
  } else {
    RCLCPP_ERROR( rclcpp::get_logger( "AdjustableOffsetTransmission" ),
                  "Failed to open transmission offset file for writing: %s",
                  transmission_file_path_.string().c_str() );
  }
}

inline void AdjustableOffsetTransmission::actuator_to_joint()
{
  // Read the current actuator position before the base transform
  if ( actuator_position_ && prev_actuator_position_.has_value() ) {
    double current_actuator_pos = actuator_position_.get_value();
    double delta = current_actuator_pos - prev_actuator_position_.value();

    // Check if the jump is close to a multiple of 2pi
    // Round to nearest integer multiple of 2pi
    double n_jumps_raw = delta / TWO_PI;
    int n_jumps = static_cast<int>( std::round( n_jumps_raw ) );

    if ( n_jumps != 0 && std::abs( delta - n_jumps * TWO_PI ) < JUMP_TOLERANCE ) {
      // Detected a 2pi jump on the actuator side.
      // Compensate in the joint offset: the jump in joint space is n_jumps * 2pi / reduction
      double joint_space_correction = n_jumps * TWO_PI / reduction_;
      jnt_offset_ -= joint_space_correction;
      correction_count_ += std::abs( n_jumps );
      saveTransmissionOffset();

      RCLCPP_WARN( rclcpp::get_logger( "AdjustableOffsetTransmission" ),
                   "Detected %d x 2pi actuator position jump on joint '%s' "
                   "(actuator delta: %.4f rad). Corrected offset to %.6f.",
                   n_jumps, joint_name_.c_str(), delta, jnt_offset_ );

      if ( offset_correction_callback_ ) {
        offset_correction_callback_( jnt_offset_ );
      }
    }
  }

  // Perform the standard transform (uses possibly-updated jnt_offset_)
  SimpleTransmission::actuator_to_joint();

  // Identity-copy any passthrough interfaces from actuator to joint side.
  for ( auto &[joint_handle, actuator_handle] : passthrough_pairs_ ) {
    if ( joint_handle && actuator_handle ) {
      joint_handle.set_value( actuator_handle.get_value() );
    }
  }

  // Track actuator position for next cycle's jump detection
  if ( actuator_position_ ) {
    prev_actuator_position_ = actuator_position_.get_value();
  }
}

inline void AdjustableOffsetTransmission::joint_to_actuator()
{
  SimpleTransmission::joint_to_actuator();

  // Identity-copy any passthrough interfaces from joint to actuator side.
  for ( auto &[joint_handle, actuator_handle] : passthrough_pairs_ ) {
    if ( joint_handle && actuator_handle ) {
      actuator_handle.set_value( joint_handle.get_value() );
    }
  }
}

inline void AdjustableOffsetTransmission::configure(
    const std::vector<transmission_interface::JointHandle> &joint_handles,
    const std::vector<transmission_interface::ActuatorHandle> &actuator_handles )
{
  passthrough_pairs_.clear();

  // Decide which interface names should bypass the base scaling.
  const auto is_passthrough_name = [this]( const std::string &name ) {
    if ( pass_through_effort_ &&
         ( name == hardware_interface::HW_IF_EFFORT || name == hardware_interface::HW_IF_TORQUE ||
           name == hardware_interface::HW_IF_FORCE ) ) {
      return true;
    }
    return std::find( pass_through_interface_names_.begin(), pass_through_interface_names_.end(),
                      name ) != pass_through_interface_names_.end();
  };

  std::vector<transmission_interface::JointHandle> filtered_joints;
  std::vector<transmission_interface::ActuatorHandle> filtered_actuators;
  std::vector<transmission_interface::JointHandle> joint_passthrough;
  std::vector<transmission_interface::ActuatorHandle> actuator_passthrough;

  filtered_joints.reserve( joint_handles.size() );
  filtered_actuators.reserve( actuator_handles.size() );

  for ( const auto &h : joint_handles ) {
    if ( is_passthrough_name( h.get_interface_name() ) ) {
      joint_passthrough.push_back( h );
    } else {
      filtered_joints.push_back( h );
    }
  }
  for ( const auto &h : actuator_handles ) {
    if ( is_passthrough_name( h.get_interface_name() ) ) {
      actuator_passthrough.push_back( h );
    } else {
      filtered_actuators.push_back( h );
    }
  }

  SimpleTransmission::configure( filtered_joints, filtered_actuators );

  // Pair passthrough handles by interface name. Drop orphans (one side missing).
  for ( const auto &joint_handle : joint_passthrough ) {
    const auto &name = joint_handle.get_interface_name();
    auto it = std::find_if( actuator_passthrough.begin(), actuator_passthrough.end(),
                            [&name]( const transmission_interface::ActuatorHandle &a ) {
                              return a.get_interface_name() == name;
                            } );
    if ( it == actuator_passthrough.end() ) {
      RCLCPP_DEBUG( rclcpp::get_logger( "AdjustableOffsetTransmission" ),
                    "Passthrough joint handle '%s' on joint '%s' has no matching actuator handle; "
                    "dropping.",
                    name.c_str(), joint_name_.c_str() );
      continue;
    }
    if ( !joint_handle || !*it ) {
      RCLCPP_DEBUG( rclcpp::get_logger( "AdjustableOffsetTransmission" ),
                    "Passthrough handle pair '%s' on joint '%s' has nullptr backing; dropping.",
                    name.c_str(), joint_name_.c_str() );
      continue;
    }
    passthrough_pairs_.emplace_back( joint_handle, *it );
  }
}

inline void AdjustableOffsetTransmission::adjustTransmissionOffset( double offset )
{
  jnt_offset_ = offset;
  saveTransmissionOffset();
  actuator_to_joint(); // update joint state based on new offset

  // Reset jump detection to avoid false positive after manual offset change
  if ( actuator_position_ ) {
    prev_actuator_position_ = actuator_position_.get_value();
  }
}
} // namespace hector_transmission_interface
#endif // DYNAMIC_OFFSET_TRANSMISSION_HPP
