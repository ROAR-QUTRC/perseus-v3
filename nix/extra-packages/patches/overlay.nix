final: prev: {
  livox-sdk2 = final.callPackage ./livox-sdk2 { };
  fast-lio = final.callPackage ./fast-lio { };
  livox-ros-driver2 = final.callPackage ./livox-ros-driver2 { };
  hector-testing-utils = final.callPackage ./hector-testing-utils { };
}
