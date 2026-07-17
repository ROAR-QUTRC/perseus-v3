{
  lib,
  stdenv,
  fetchFromGitHub,
  cmake,
}:

stdenv.mkDerivation {
  pname = "livox-sdk2";
  version = "1.3.1";

  src = fetchFromGitHub {
    owner = "Livox-SDK";
    repo = "Livox-SDK2";
    rev = "v1.3.1";
    sha256 = "sha256-XM2jhytXbLVd3jkeZrpxDjegPWPiXCaVQ3nYm1DD928=";
  };

  patches = [
    # ./patches/fix-cstdint-and-enum.patch
  ];

  nativeBuildInputs = [ cmake ];

  CXXFLAGS = [
    "-std=c++17"
    "-include cstdint"
    # "-Wno-error=cpp"
    # "-Wno-error=deprecated-declarations"
    # "-Wno-c++20-compat" # Suppress char8_t warning from spdlog/fmt
    # "-fPIC"
  ];

  cmakeFlags = [
    "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
    # "-DCMAKE_BUILD_TYPE=Release"
    # "-DCMAKE_INSTALL_PREFIX=${placeholder "out"}"
    # "-DCMAKE_CXX_STANDARD=17"
    # "-DCMAKE_CXX_STANDARD_REQUIRED=ON"
    # "-DCMAKE_CXX_EXTENSIONS=OFF"
    # "-DCMAKE_POSITION_INDEPENDENT_CODE=ON"
  ];

  meta = with lib; {
    description = "Livox SDK 2 for LiDAR devices";
    homepage = "https://github.com/Livox-SDK/Livox-SDK2";
    license = licenses.mit;
    platforms = platforms.linux;
    mainProgram = "livox_lidar_sdk_shared";
  };
}
