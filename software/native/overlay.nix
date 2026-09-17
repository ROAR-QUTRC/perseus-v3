final: prev: {
  examples = final.lib.packagesFromDirectoryRecursive {
    inherit (final) callPackage;
    directory = ./examples;
  };
  camera-server = final.lib.packagesFromDirectoryRecursive {
    inherit (final) callPackage;
    directory = ./camera_server;
  };
}
