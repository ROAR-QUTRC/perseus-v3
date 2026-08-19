rosDistro: final: prev:
let
  regularPackages = final: prev: {
    livox-sdk2 = final.callPackage ./livox-sdk2 { };
  };
  colcon = (import ./colcon/overlay.nix);
  ros-packages = (import ./ros-packages/overlay.nix rosDistro);

  composed = prev.lib.composeManyExtensions [
    regularPackages
    colcon
    ros-packages
  ] final prev;
in
composed
