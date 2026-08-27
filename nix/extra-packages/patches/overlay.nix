final: prev: {
  # BIEVR-LIO builds three packages out of one repository, all pinned to the same commit
  # by bievr-lio/src.nix.
  bievr-lio = final.callPackage ./bievr-lio { };
  bievr-lio-ros2 = final.callPackage ./bievr-lio-ros2 { };
  bievr-ros-common = final.callPackage ./bievr-ros-common { };
  # Base-station meshing: reads the decompressed map cloud and reconstructs a surface.
  immesh-ros2 = final.callPackage ./immesh-ros2 { };
  livox-sdk2 = final.callPackage ./livox-sdk2 { };
  fast-lio = final.callPackage ./fast-lio { };
  livox-ros-driver2 = final.callPackage ./livox-ros-driver2 { };
}
