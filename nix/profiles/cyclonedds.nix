# A dev shell that points CycloneDDS at a config generated from config/network_devices.toml.
#
# Identical to `dev` in every other respect, since it extends it -- the only
# difference is CYCLONEDDS_URI. RMW_IMPLEMENTATION is already rmw_cyclonedds_cpp
# for every profile (see postShellHook in nix/ros-workspace.nix), so the config
# applies to every ROS process started from this shell, with no per-node setup.
#
# The config turns multicast off and names every enabled device as a peer explicitly.
# That is what the field network needs: on a link where multicast is dropped or floods,
# discovery otherwise never converges, and the symptom is nodes that start cleanly and
# simply never see each other.
{ pkgs, lib, ... }:
let
  devices = builtins.fromTOML (builtins.readFile ../../config/network_devices.toml);
  peers = lib.filterAttrs (_: device: device.enabled or true) devices;
  peerAddresses = lib.mapAttrsToList (_: device: device.ip) peers;

  # Which interface to bind to. Cyclone 0.10 takes exactly one name: there is no
  # wildcard, and listing several as fallbacks fails as a whole if any single one of
  # them is absent. A name that does not resolve is fatal rather than degraded -- the
  # daemon reports "No network interface selected" and every rmw_create_node call fails,
  # so nodes do not start at all.
  #
  # The name therefore has to be right per device. It is supplied by ROAR_DDS_IFACE, which
  # Cyclone expands itself and nix/profiles/hostnames.nix sets from the device table; a
  # machine missing from the table takes the eth0 default.
  #
  # NetworkInterfaceAddress used to spell this and still parses, but 0.10 reports it as a
  # deprecated element and rewrites it to the form below.
  ddsConfig = pkgs.writeText "cyclonedds.xml" ''
    <CycloneDDS>
      <Domain>
        <General>
          <AllowMulticast>false</AllowMulticast>
          <Interfaces>
            <NetworkInterface name="''${ROAR_DDS_IFACE:-eth0}"/>
          </Interfaces>
        </General>
        <Discovery>
          <Peers>
    ${lib.concatMapStringsSep "\n" (
      name: "        <!-- ${name} -->\n        <Peer address=\"${peers.${name}.ip}\"/>"
    ) (lib.attrNames peers)}
          </Peers>
        </Discovery>
      </Domain>
    </CycloneDDS>
  '';

  # The address each device is listed at, so the shell can check this machine really holds it.
  ipByHost = lib.concatStrings (
    lib.mapAttrsToList (name: device: ''
      "${name}") dds_ip=${device.ip} ;;
    '') devices
  );
in
{
  env.CYCLONEDDS_URI = "file://${ddsConfig}";

  enterShell = ''
    echo -e "\e[38;5;208mCycloneDDS: unicast discovery via config/network_devices.toml\e[0m"

    # A name that does not resolve is fatal to every node (see ddsConfig above), but that
    # lands per node at launch, far from the config that caused it, so check the same
    # name here where the cause is still in front of you.
    dds_host=$(hostname)
    dds_ip=
    case "$dds_host" in
    ${ipByHost}
    esac
    dds_iface=''${ROAR_DDS_IFACE:-eth0}

    if [ -z "$dds_ip" ]; then
      echo -e "\e[33m  WARNING: $dds_host is not in config/network_devices.toml.\e[0m"
      echo    "           Binding to the eth0 default, and no other machine lists this one as a peer."
    fi

    if ! ip link show "$dds_iface" >/dev/null 2>&1; then
      dds_avail=$(ip -brief link show | awk '$2 == "UP" { printf "%s ", $1 }')
      echo -e "\e[33m  WARNING: interface '$dds_iface' does not exist on this machine.\e[0m"
      echo    "           Every ROS node will fail to start: rmw_create_node -> failed to create domain."
      echo    "           Interfaces that are up here: $dds_avail"
      echo    "           Fix the entry for $dds_host in config/network_devices.toml."
    else
      echo    "  interface: $dds_iface"
      # The other machines find this one only at the address the table gives it. If the
      # interface holds a different one -- a DHCP lease that moved, say -- this machine
      # still starts cleanly and simply never shows up anywhere else.
      if [ -n "$dds_ip" ] && ! ip -4 -brief addr show dev "$dds_iface" \
        | grep -qwF "$dds_ip"; then
        dds_held=$(ip -4 -brief addr show dev "$dds_iface" | awk '{ $1 = $2 = ""; print }')
        echo -e "\e[33m  WARNING: $dds_iface does not hold $dds_ip, the address listed for $dds_host.\e[0m"
        echo    "           It holds:$dds_held"
        echo    "           Other machines will not discover this one until the two agree."
      fi
    fi
    echo    "  peers:     ${lib.concatStringsSep " " peerAddresses}"
  '';
}
