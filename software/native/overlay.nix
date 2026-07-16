final: prev: {
  examples = final.lib.packagesFromDirectoryRecursive {
    inherit (final) callPackage;
    directory = ./examples;
  };
}
