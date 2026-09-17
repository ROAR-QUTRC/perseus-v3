# A dev shell that runs ROS on Zenoh instead of CycloneDDS.
#
# Identical to `dev` in every other respect, since it extends it -- the only difference is
# the middleware: RMW_IMPLEMENTATION plus the two Zenoh config files. Every other profile
# keeps rmw_cyclonedds_cpp, which postShellHook in nix/ros-workspace.nix sets.
#
# Zenoh is broker-based where DDS is peer-to-peer. Nodes do not discover each other: they
# all connect to one rmw_zenohd router process, which matches publishers to subscribers and
# is the only thing that relays between hosts. Nothing here starts it for you:
#
#     ros2 run rmw_zenoh_cpp rmw_zenohd
#
# That has to be running before any node does, and it reads ZENOH_ROUTER_CONFIG_URI on its
# own -- no -c flag. The session config sets mode "client" with connect.exit_on_failure.client
# true, so a node that cannot reach a router exits rather than degrading. The symptom of a
# missing router is every node dying at launch, not a stack that comes up mute.
#
# Which machine runs the router is a real decision, not a detail. Clients route *everything*
# through it, so it belongs on the robot rather than a base station: put it on the base
# station and the rover's own perception-to-control traffic crosses the radio link twice,
# and a dropout kills every node on the robot instead of just the operator's RViz.
{
  pkgs,
  config,
  lib,
  ...
}:
let
  networkingDir = "${config.env.DEVENV_ROOT}/software/networking";
  sessionConfig = "${networkingDir}/zenoh_session_config.json5";
  routerConfig = "${networkingDir}/zenoh_router_config.json5";
in
{
  packages =
    (with pkgs; [
      # Standalone zenoh, for the z_* tools (z_scout, z_ping, z_sub) that let you test a
      # link with no ROS in the way. Note its `zenohd` is NOT the router to use here: it
      # ignores ZENOH_ROUTER_CONFIG_URI entirely and takes -c instead, so starting it by
      # mistake gives you a router on stock settings that looks like it worked.
      zenoh
    ])
    ++ (with pkgs.rosPackages.jazzy; [
      rmw-zenoh-cpp
      zenoh-cpp-vendor
    ]);

  # Paths under DEVENV_ROOT rather than the nix store, matching how the cyclonedds profile
  # points at its XML. Both processes read the file at startup, so editing one takes effect
  # on the next launch instead of on the next `devenv shell`.
  env = {
    ZENOH_SESSION_CONFIG_URI = sessionConfig;
    ZENOH_ROUTER_CONFIG_URI = routerConfig;
  };

  # mkAfter because dev's enterShell exports RMW_IMPLEMENTATION=rmw_cyclonedds_cpp through
  # the inlined nix-ros-workspace shellHook, and the last export wins. Left to module merge
  # order this would be a coin flip, and losing it is silent: the banner below still says
  # Zenoh while every node speaks DDS.
  enterShell = lib.mkAfter ''
    export LD_LIBRARY_PATH="${
      lib.makeLibraryPath (
        with pkgs.rosPackages.jazzy;
        [
          rmw-zenoh-cpp
          zenoh-cpp-vendor
        ]
      )
    }:$LD_LIBRARY_PATH"

    export RMW_IMPLEMENTATION=rmw_zenoh_cpp

    echo -e "\e[38;5;208mZenoh: rmw_zenoh_cpp via software/networking/zenoh_*_config.json5\e[0m"

    # Report the endpoints a node will dial, since that is the setting that decides whether
    # this machine talks to anything, and it is buried 40 lines into an 819-line file.
    # connect.endpoints is the first `endpoints:` in the file; listen.endpoints follows it.
    zenoh_connect=$(sed -n 's/^[[:space:]]*endpoints: \[\(.*\)\],[[:space:]]*$/\1/p' \
      ${sessionConfig} | head -n1)
    if [ -z "$zenoh_connect" ]; then
      echo -e "\e[33m  WARNING: no connect.endpoints found in zenoh_session_config.json5.\e[0m"
      echo    "           Nodes in client mode exit at startup with nowhere to connect."
    else
      echo    "  router:   $zenoh_connect"
    fi
    echo    "  start it: ros2 run rmw_zenoh_cpp rmw_zenohd   (before any node, in its own shell)"
  '';
}
