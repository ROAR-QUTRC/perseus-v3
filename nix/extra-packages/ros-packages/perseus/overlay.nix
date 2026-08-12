final: prev: {
  autonomy = final.callPackage ./autonomy.nix { };
  perseus = final.callPackage ./perseus.nix { };
  perseus-can-if = final.callPackage ./perseus-can-if.nix { };
  perseus-description = final.callPackage ./perseus-description.nix { };
  perseus-hardware = final.callPackage ./perseus-hardware.nix { };
  perseus-interfaces = final.callPackage ./perseus-interfaces.nix { };
  perseus-payloads = final.callPackage ./perseus-payloads.nix { };
  perseus-sensors = final.callPackage ./perseus-sensors.nix { };
  perseus-teleop = final.callPackage ./perseus-teleop.nix { };
  perseus-vision = final.callPackage ./perseus-vision.nix { };
}
