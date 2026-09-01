final: prev: {
  arena-server = final.callPackage ./arena-server.nix { };
  autonomy-bringup = final.callPackage ./autonomy-bringup.nix { };
  can-if = final.callPackage ./can-if.nix { };
  description = final.callPackage ./description.nix { };
  footprint-broadcaster = final.callPackage ./footprint-broadcaster.nix { };
  global-traversability = final.callPackage ./global-traversability.nix { };
  hardware = final.callPackage ./hardware.nix { };
  health-check = final.callPackage ./health-check.nix { };
  interfaces = final.callPackage ./interfaces.nix { };
  payloads = final.callPackage ./payloads.nix { };
  perseus = final.callPackage ./perseus.nix { };
  rviz-plugins = final.callPackage ./rviz-plugins.nix { };
  sensors = final.callPackage ./sensors.nix { };
  teleop = final.callPackage ./teleop.nix { };
  vision = final.callPackage ./vision.nix { };
  watchdog = final.callPackage ./watchdog.nix { };
}
