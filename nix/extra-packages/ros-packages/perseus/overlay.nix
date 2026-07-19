final: prev: {
  perseus-can-if = final.callPackage ./perseus-can-if.nix { };
  template-package = final.callPackage ./template-package.nix { };
}
