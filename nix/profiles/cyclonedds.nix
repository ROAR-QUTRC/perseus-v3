# A dev shell that points CycloneDDS at config/cyclonedds_autonomy.xml.
#
# Identical to `dev` in every other respect, since it extends it -- the only
# difference is CYCLONEDDS_URI. RMW_IMPLEMENTATION is already rmw_cyclonedds_cpp
# for every profile (see postShellHook in nix/ros-workspace.nix), so the config
# applies to every ROS process started from this shell, with no per-node setup.
#
# The config turns multicast off and names the rover and base station peers
# explicitly. That is what the field network needs: on a link where multicast is
# dropped or floods, discovery otherwise never converges, and the symptom is nodes
# that start cleanly and simply never see each other.
{ config, ... }:
let
  ddsConfig = "${config.env.DEVENV_ROOT}/config/cyclonedds_autonomy.xml";
in
{
  # file:// plus an absolute path gives the canonical file:///... form Cyclone wants.
  env.CYCLONEDDS_URI = "file://${ddsConfig}";

  enterShell = ''
    echo -e "\e[38;5;208mCycloneDDS: unicast discovery via config/cyclonedds_autonomy.xml\e[0m"

    # The config names a single interface. Cyclone refuses to initialise when that
    # interface is absent, and on this path the failure does not present as an error
    # at shell entry -- it presents much later as every topic silently missing. Worth
    # one check up front.
    dds_iface=$(sed -n 's:.*<NetworkInterfaceAddress>\(.*\)</NetworkInterfaceAddress>.*:\1:p' \
      ${ddsConfig} | head -n1 | tr -d '[:space:]')
    if [ -n "$dds_iface" ] && ! ip link show "$dds_iface" >/dev/null 2>&1; then
      dds_avail=$(ip -brief link show | awk '$2 == "UP" { printf "%s ", $1 }')
      echo -e "\e[33m  WARNING: interface '$dds_iface' does not exist on this machine.\e[0m"
      echo    "           CycloneDDS will fail to start and no ROS node will see any other."
      echo    "           Interfaces that are up here: $dds_avail"
      echo    "           Edit NetworkInterfaceAddress in config/cyclonedds_autonomy.xml,"
      echo    "           or unset CYCLONEDDS_URI to fall back to default discovery."
    else
      echo    "  interface: $dds_iface"
      echo    "  peers:     $(grep -c '<Peer ' ${ddsConfig}) enabled"
    fi
  '';
}
