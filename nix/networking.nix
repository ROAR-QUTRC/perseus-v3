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
  '';
}
