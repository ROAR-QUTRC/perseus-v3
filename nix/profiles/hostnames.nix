# Per-machine settings, keyed by hostname.
#
# devenv resolves profiles.hostname.<hostname> against the current host automatically,
# ahead of the user profile and of whatever was named with --profile, and gives it the
# strongest priority of the three. So a value set here wins, and no one has to remember
# to pass an extra flag on a particular machine.
#
# Right now this exists for ROAR_DDS_IFACE: config/cyclonedds_autonomy.xml binds Cyclone
# to exactly one interface name, and a name that does not resolve stops every ROS node
# from starting. See the comment in that file.
{
  # Kelvin's laptop, on wifi: wlp18s0 holds 192.168.2.13 and carries the route to the
  # rover subnet. It has been wired before, as enp12s0 on 192.168.2.21, and both have sat
  # on 192.168.2.0/24 at once -- which is why this is pinned rather than left to Cyclone's
  # autodetection. Switching back to the dock means changing this and the matching <Peer>
  # entry in config/cyclonedds_autonomy.xml together, since the address moves with it.
  BoMachine.module = {
    env.ROAR_DDS_IFACE = "wlp18s0";
  };

  # big-brain, medium-brain and small-brain take the eth0 default from
  # config/cyclonedds_autonomy.xml and so need no entry here. Add one in this shape if
  # `ip -brief link` on any of them ever reports something else -- Jetson carrier boards
  # in particular tend to name the port after its PCIe address rather than eth0:
  #
  #   "big-brain".module = {
  #     env.ROAR_DDS_IFACE = "enP8p1s0";
  #   };
}
