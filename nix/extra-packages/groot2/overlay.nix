# Exposes the Groot2 behavior tree viewer as pkgs.groot2
final: prev: {
  groot2 = final.callPackage ./package.nix { };
}
