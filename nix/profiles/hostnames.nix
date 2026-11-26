# Per-machine settings, keyed by hostname.
#
# devenv resolves profiles.hostname.<hostname> against the current host automatically,
# ahead of the user profile and of whatever was named with --profile, and gives it the
# strongest priority of the three. So a value set here wins, and no one has to remember
# to pass an extra flag on a particular machine.
#
# The network part is generated from config/network_devices.toml: each device there gets
# ROAR_DDS_IFACE set to its `interface`, which the CycloneDDS config binds to. Add or
# change machines in that file, not here. Per-host settings unrelated to the network can
# be merged in below.
{ lib }:
let
  devices = builtins.fromTOML (builtins.readFile ../../config/network_devices.toml);
in
lib.mapAttrs (_: device: {
  module = {
    env.ROAR_DDS_IFACE = device.interface;
  };
}) devices
