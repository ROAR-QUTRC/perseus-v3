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

    # Cyclone 0.10 binds to exactly one interface name: no wildcard, and a list whose
    # entries do not all resolve fails as a whole. A name that does not resolve is fatal
    # rather than degraded -- "No network interface selected", and every rmw_create_node
    # fails. That lands per node at launch, far from the config that caused it, so resolve
    # the same name here where the cause is still in front of you.
    dds_spec=$(sed -n 's:.*<NetworkInterface[^>]*name="\([^"]*\)".*:\1:p' \
      ${ddsConfig} | head -n1)
    # Mirror Cyclone's own expansion: the default is whatever follows ":-". A plain name
    # with no variable leaves this empty and falls through to the spec itself.
    dds_iface=$(printf '%s' "$dds_spec" | sed -n 's/.*:-\(.*\)}$/\1/p')
    if [ -n "$ROAR_DDS_IFACE" ]; then
      dds_iface=$ROAR_DDS_IFACE
      dds_origin="ROAR_DDS_IFACE, set by hostname profile $(hostname)"
    else
      dds_origin="default in config/cyclonedds_autonomy.xml"
    fi
    if [ -z "$dds_iface" ]; then
      dds_iface=$dds_spec
    fi

    if [ -n "$dds_iface" ] && ! ip link show "$dds_iface" >/dev/null 2>&1; then
      dds_avail=$(ip -brief link show | awk '$2 == "UP" { printf "%s ", $1 }')
      echo -e "\e[33m  WARNING: interface '$dds_iface' does not exist on this machine.\e[0m"
      echo    "           Every ROS node will fail to start: rmw_create_node -> failed to create domain."
      echo    "           Interfaces that are up here: $dds_avail"
      echo    "           Add an entry for $(hostname) to nix/profiles/hostnames.nix."
    else
      echo    "  interface: $dds_iface ($dds_origin)"
      echo    "  peers:     $(grep -c '<Peer ' ${ddsConfig}) enabled"
    fi
  '';
}
