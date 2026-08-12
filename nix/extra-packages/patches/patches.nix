rosDistro: final: prev:
let
  rosOverlay = rosFinal: rosPrev: {
    livox-ros-driver2 = rosPrev.livox-ros-driver2.overrideAttrs (
      {
        buildInputs ? [ ],
        patches ? [ ],
        ...
      }:
      {
        buildInputs = buildInputs ++ [ rosFinal.livox-sdk2 ];
        patches = patches ++ [
          ./livox-ros-driver2/rename-files.patch
          ./livox-ros-driver2/livox-ros-driver2.patch
        ];
      }
    );
    fast-lio = rosPrev.fast-lio.overrideAttrs (
      {
        patches ? [ ],
        ...
      }:
      {
        # We need to have submodules, so we should use fetchGit instead
        src = builtins.fetchGit {
          url = "https://github.com/hku-mars/FAST_LIO";
          ref = "ROS2";
          narHash = "sha256-chnAIRkSQjoXqg9K9s1JVOrNdFtEzFztOFUYnbXkZyI=";
          rev = "a4743b095409588842a5b30ddfa27e29d2f99164";
          submodules = true;
        };
        # Fast-LIO sets the cpp standard to 14, but jazzy needs version 17
        patches = patches ++ [
          ./fast-lio/cpp_version_17.patch
          ./fast-lio/frame_id_fix.patch
        ];
      }
    );
    perseus-input = rosPrev.perseus-input.overrideAttrs (
      {
        propagatedBuildInputs ? [ ],
        ...
      }:
      {
        propagatedBuildInputs = final.lib.remove rosFinal.perseus-input-config propagatedBuildInputs;
      }
    );
    perseus-vision =
      let
        cubeDetectorModel = prev.fetchurl {
          url = "https://github.com/ROAR-QUTRC/perseus-v2/releases/download/models-v1/cube_detector_yolob8s.onnx";
          sha256 = "sha256-EkWhKFYog5ysSobcE4DFW2S8j3ZLQZDBxucWLa/KVfc=";
        };
      in
      rosPrev.perseus-vision.overrideAttrs (
        {
          postUnpack ? "",
          ...
        }:
        {
          postUnpack = postUnpack + ''
            mkdir -p $sourceRoot/models
            cp ${cubeDetectorModel} $sourceRoot/models/cube_detector_yolob8s.onnx
          '';
        }
      );
  };
in
{
  rosPackages = prev.rosPackages // {
    # we need to use overrideScope and an overlay to apply the changes
    # so that they propagate properly
    ${rosDistro} = prev.rosPackages.${rosDistro}.overrideScope rosOverlay;
  };
}
