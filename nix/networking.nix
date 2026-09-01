{
  pkgs,
  lib,
  ...
}:

{

  packages =
    (with pkgs; [
      zenoh
    ])
    ++ (with pkgs.rosPackages.jazzy; [
      rmw-zenoh-cpp
      zenoh-cpp-vendor
    ]);

  enterShell = ''
    export LD_LIBRARY_PATH="${
      lib.makeLibraryPath (
        with pkgs.rosPackages.jazzy;
        [
          rmw-zenoh-cpp
          zenoh-cpp-vendor
        ]
      )
    }:$LD_LIBRARY_PATH"

    # use Zenoh ROS middleware
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp

    # our custom Zenoh configs
    export ZENOH_SESSION_CONFIG_URI="${../software/networking/zenoh_session_config.json5}"
    export ZENOH_ROUTER_CONFIG_URI="${../software/networking/zenoh_router_config.json5}"

    # debug for Zenoh
    export RUST_LOG=zenoh=debug
  '';
}
