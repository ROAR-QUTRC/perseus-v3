# Every machine that runs a Zenoh router, keyed by its hostname (what `hostname` prints).
#
# This is the one place to edit when a machine joins or leaves the network. ROS nodes never
# leave their own machine: they connect only to the router on localhost, and the routers
# carry traffic between machines.
#
# Machines are grouped into sites (the rover and the base station). Inside a site every
# router dials every other router, a full mesh. Between sites, only the bridges talk: each
# site has at most one host with `bridge = true`, and the bridges dial each other. Every
# other router reaches the far site through its own site's bridge. So the rover-to-base link
# carries exactly one router connection, big-brain <-> cookie.
#
# A machine that is not listed joins the site named by $ZENOH_SITE (default "base") as a
# non-bridge. Its router dials that site's hosts, but no other router dials it.
#
# Later, after rmw_zenoh moves to zenoh >= 1.9, `site` maps to Zenoh's `region_name` and the
# bridges become region gateways. See software/networking/README.md.
#
# Fields:
#   ip     - address the other routers dial to reach this host.
#   site   - required: "rover" or "base" (any name works; hosts with the same name mesh).
#   bridge - optional, default false: this host carries its site's link to other sites.
#   port   - optional, default 7447. Must match listen/endpoints in
#            zenoh_router_config.json5 on that host.
{
  # --- Rover ---
  "big-brain" = {
    ip = "192.168.2.11";
    site = "rover";
    bridge = true;
  };
  "medium-brain" = {
    ip = "192.168.2.12";
    site = "rover";
  };
  "small-brain" = {
    ip = "192.168.2.15";
    site = "rover";
  };

  # --- Base station ---
  # Dan's machine
  cookie = {
    ip = "192.168.2.50";
    site = "base";
    bridge = true;
  };

  # Kelvin's laptop, on wifi (wlp18s0). When docked it is enp12s0 on 192.168.2.21 instead.
  BoMachine = {
    ip = "192.168.2.13";
    site = "base";
  };

  # Luca's machine
  # <hostname> = { ip = "192.168.2.14"; site = "base"; };
  # Shaan's machine
  # <hostname> = { ip = "192.168.2.140"; site = "base"; };
}
