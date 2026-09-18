
#include "transmission_interface/simple_transmission_loader.hpp"

#include <algorithm>
#include <cctype>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "transmission_interface/simple_transmission.hpp"
#include <hector_transmission_interface/adjustable_offset_transmission_loader.hpp>

namespace hector_transmission_interface
{

namespace
{
bool parseBoolParam( const std::string &value )
{
  std::string lower;
  lower.reserve( value.size() );
  std::transform( value.begin(), value.end(), std::back_inserter( lower ),
                  []( unsigned char c ) { return std::tolower( c ); } );
  return lower == "true" || lower == "1" || lower == "yes" || lower == "on";
}

std::vector<std::string> parseInterfaceList( const std::string &value )
{
  // Split on commas or whitespace; trim each entry; drop empties.
  std::vector<std::string> result;
  std::string current;
  const auto flush = [&]() {
    auto begin = current.find_first_not_of( " \t" );
    auto end = current.find_last_not_of( " \t" );
    if ( begin != std::string::npos ) {
      result.emplace_back( current.substr( begin, end - begin + 1 ) );
    }
    current.clear();
  };
  for ( const char c : value ) {
    if ( c == ',' || c == ' ' || c == '\t' ) {
      flush();
    } else {
      current.push_back( c );
    }
  }
  flush();
  return result;
}
} // namespace

std::shared_ptr<transmission_interface::Transmission> AdjustableOffsetTransmissionLoader::load(
    const hardware_interface::TransmissionInfo &transmission_info )
{
  try {
    const auto &joint_info = transmission_info.joints.at( 0 );
    const auto mechanical_reduction = joint_info.mechanical_reduction;
    const auto offset = joint_info.offset;
    const auto joint_name = joint_info.name;

    bool pass_through_effort = false;
    std::vector<std::string> pass_through_interfaces;
    const auto pte_it = transmission_info.parameters.find( "pass_through_effort" );
    if ( pte_it != transmission_info.parameters.end() ) {
      pass_through_effort = parseBoolParam( pte_it->second );
    }
    const auto pti_it = transmission_info.parameters.find( "pass_through_interfaces" );
    if ( pti_it != transmission_info.parameters.end() ) {
      pass_through_interfaces = parseInterfaceList( pti_it->second );
    }

    std::shared_ptr<transmission_interface::Transmission> transmission(
        new AdjustableOffsetTransmission( joint_name, mechanical_reduction, offset,
                                          pass_through_effort,
                                          std::move( pass_through_interfaces ) ) );
    return transmission;
  } catch ( const std::exception &ex ) {
    RCLCPP_ERROR( rclcpp::get_logger( "simple_transmission_loader" ),
                  "Failed to construct transmission '%s'", ex.what() );
    return {};
  }
}

} // namespace hector_transmission_interface

PLUGINLIB_EXPORT_CLASS( hector_transmission_interface::AdjustableOffsetTransmissionLoader,
                        transmission_interface::TransmissionLoader )
