rosDistro: final: prev:
prev.lib.composeManyExtensions [
  (import ./fast-lio/overlay.nix)
  (import ./livox-ros-driver2/overlay.nix)
] final prev
