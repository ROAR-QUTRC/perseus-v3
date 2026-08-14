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
    perseus-input = rosPrev.perseus-input.overrideAttrs (
      {
        propagatedBuildInputs ? [ ],
        ...
      }:
      {
        propagatedBuildInputs = final.lib.remove rosFinal.perseus-input-config propagatedBuildInputs;
      }
    );
    vision =
      let
        cubeDetectorModel = prev.fetchurl {
          url = "https://github.com/ROAR-QUTRC/perseus-v2/releases/download/models-v1/cube_detector_yolob8s.onnx";
          sha256 = "sha256-EkWhKFYog5ysSobcE4DFW2S8j3ZLQZDBxucWLa/KVfc=";
        };
      in
      rosPrev.vision.overrideAttrs (
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
