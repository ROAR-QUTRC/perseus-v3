# Do not include perseus packages in here. They are defined in the ros_ws overlay
rosDistro: final: prev: {
  fast-lio = final.callPackage ./fast-lio { };
  livox-ros-driver2 = final.callPackage ./livox-ros-driver2 { };
}
