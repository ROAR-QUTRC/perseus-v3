{ pkgs, ... }:
{
  # zenoh CLI tools (zenohd, z_*); rmw-zenoh-cpp comes from the ROS workspace
  packages = [ pkgs.zenoh ];

  enterShell = ''
    # use Zenoh ROS middleware
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp

    # our custom Zenoh configs
    export ZENOH_SESSION_CONFIG_URI="${../software/networking/zenoh_session_config.json5}"
    export ZENOH_ROUTER_CONFIG_URI="${../software/networking/zenoh_router_config.json5}"

    # Zenoh log level for ROS nodes. The router sets its own (ZENOH_ROUTER_LOG, see
    # nix/profiles/zenoh.nix). Set RUST_LOG=zenoh=debug before entering to debug.
    export RUST_LOG=''${RUST_LOG:-zenoh=warn}
  '';
}
