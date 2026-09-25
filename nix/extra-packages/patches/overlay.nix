final: prev: {
  # BIEVR-LIO builds three packages out of one repository, all pinned to the same commit
  # by bievr-lio/src.nix.
  bievr-lio = final.callPackage ./bievr-lio { };
  bievr-lio-ros2 = final.callPackage ./bievr-lio-ros2 { };
  bievr-ros-common = final.callPackage ./bievr-ros-common { };
  livox-sdk2 = final.callPackage ./livox-sdk2 { };
  livox-ros-driver2 = final.callPackage ./livox-ros-driver2 { };
}
