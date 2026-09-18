#ifndef HECTOR_TRANSMISSION_INTERFACE__DYNAMIC_OFFSET_TRANSMISSION_LOADER_HPP_
#define HECTOR_TRANSMISSION_INTERFACE__DYNAMIC_OFFSET_TRANSMISSION_LOADER_HPP_

#include <memory>

#include <hector_transmission_interface/adjustable_offset_transmission.hpp>
#include <transmission_interface/transmission.hpp>
#include <transmission_interface/transmission_loader.hpp>
namespace hector_transmission_interface
{
/**
 * \brief Class for loading a simple transmission instance from configuration data.
 */
class AdjustableOffsetTransmissionLoader : public transmission_interface::TransmissionLoader
{
public:
  std::shared_ptr<transmission_interface::Transmission>
  load( const hardware_interface::TransmissionInfo &transmission_info ) override;
};

} // namespace hector_transmission_interface

#endif // HECTOR_TRANSMISSION_INTERFACE__DYNAMIC_OFFSET_TRANSMISSION_LOADER_HPP_
