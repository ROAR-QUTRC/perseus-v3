# CSIRO's OHM (Occupancy Homogeneous Map): probabilistic 3D voxel occupancy mapping, plus
# the 2.5D heightmap built on top of it. Not a ROS package -- plain CMake, no rclcpp
# anywhere in the tree -- so it is packaged here as an ordinary library and consumed by
# ohm_mapping, which is the ROS node this repo writes around it.
#
# BUILT CPU-ONLY, DELIBERATELY. OHM exists to put ray integration on a GPU, and upstream
# says so: docs/gpu/docgpudetail.md is explicit that the GPU algorithms "perform
# significantly better than single threaded CPU algorithms", and the CPU ray mapper
# (ohm/RayMapperOccupancy.cpp:68) is exactly that -- a serial loop. Neither GPU backend is
# reachable on the hardware this runs on:
#
#   CUDA    RK3588 is not an NVIDIA part. Nothing to enable.
#   OpenCL  Mali-G610 would need either the proprietary blob or Panfrost's incomplete CL,
#           and neither is packaged in nixpkgs for aarch64 in a form that survives a
#           hermetic build.
#
# What makes CPU-only viable here anyway is the point rate, which is an order of magnitude
# below what the feasibility study assumed. The MID-360's datasheet rate is ~200k points/s,
# but measured on rosbag2_1970_01_01-10_06_09 only 10.0% of the cloud survives: 87.4% are
# zero-fill non-returns, 2.6% fall inside lidar.min_range_m, 0.04% beyond max_range_m. So
# integration sees ~20k points/s, and the ~30M voxel updates/s that made CPU-only hopeless
# becomes ~3M.
#
# The heightmap is the piece that actually costs, and it is unconditionally serial:
# ohmheightmap/Heightmap.h:146 declares setThreadCount() and nothing in the repository
# defines it, so calling it is a link error. It also rebuilds from scratch every call
# (Heightmap.cpp:534 clears the map first), and its cost scales with map AREA rather than
# point count -- which is why ohm_mapping always passes a cull box.
{
  lib,
  stdenv,
  fetchFromGitHub,
  cmake,
  python3,
  glm,
  zlib,
  eigen,
  tbb,
}:
stdenv.mkDerivation {
  pname = "ohm";
  version = "0.5.0";

  # The ROAR fork rather than csiro-robotics/ohm directly, so a pin can carry local fixes
  # if any are ever needed. It is currently identical to upstream master.
  src = fetchFromGitHub {
    owner = "ROAR-QUTRC";
    repo = "ohm";
    rev = "4e2e76945e550eb8a96cef9c4434e7c1811dd030";
    hash = "sha256-pL3pWTD8+6It0TOEsnVrx70ML3wA0q7jAoEYwzGhDpE=";
  };

  # OHM is 2023 code and does not build against a 2026 toolchain untouched. Both patches
  # are compatibility only -- no behaviour is changed by either.
  patches = [
    # cmake/Findglm.cmake predates glm 1.0's split of glm::glm into a STATIC target plus
    # glm::glm-header-only, and reads the include directories off the one that no longer
    # carries them. Configuration fails outright without this.
    ./findglm-1.0-header-only-target.patch
    # Three headers that used to arrive transitively and no longer do: <utility> for
    # std::exchange (libstdc++ 13 pruned its internal includes) and glm/gtc/quaternion.hpp
    # for glm::dquat (glm 1.0 stopped pulling it in via glm/gtx/norm.hpp).
    ./missing-includes.patch
  ];

  # python3 is not used at runtime and OHM does not build any Python. It is here because
  # cmake/ClangTidy.cmake:44 does an unconditional find_package(Python3 REQUIRED) -- the
  # module is included from CMakeLists.txt:77 with no guard, so configuration fails outright
  # without an interpreter present, whether or not clang-tidy itself is ever run (it is not:
  # no clang-tidy binary is in this build, so CLANG_TIDY_OK ends up false and the lint
  # targets are simply never created).
  nativeBuildInputs = [
    cmake
    python3
  ];
  # glm and TBB are PROPAGATED, and that is not tidiness. The installed
  # share/ohm/cmake/ohm-packages.cmake re-runs find_package(glm REQUIRED) and
  # find_package(TBB CONFIG) inside every consumer's configure step, so a dependent whose
  # CMAKE_PREFIX_PATH lacks them fails at find_package(ohm) with "Could not find a package
  # configuration file provided by glm" -- pointing at the consumer, not at this file. glm
  # is also part of the public interface outright: OccupancyMap and Heightmap take
  # glm::dvec3 across the API.
  propagatedBuildInputs = [
    glm
    tbb
  ];

  # These two stay private. Shared libraries, so ohm-packages.cmake only re-finds ZLIB
  # when OHM_BUILD_SHARED is off, and Eigen is used internally by the covariance
  # eigen-decomposition without appearing in any installed header's interface.
  buildInputs = [
    zlib
    eigen
  ];

  # The same libstdc++ 13 pruning as the patch above, but spread across enough headers
  # (MapLayout.h, Trace.h, VoxelBlockCompressionQueue.h and more, all declaring uint16_t /
  # uint64_t members) that patching each one individually would be a large and fragile diff
  # for no extra correctness. Same approach, and same reason, as livox-sdk2's -include
  # cstdint in this directory. C++ only: nothing here compiles as C.
  env.CXXFLAGS = "-include cstdint";

  cmakeFlags = [
    # C++14 is the project default (CMakeLists.txt:6). Raised to 17 to match everything
    # else in this workspace -- ohm_mapping is C++20 and cannot link a C++14 library that
    # exposes std::string/std::vector across the ABI boundary without both agreeing.
    "-DCMAKE_CXX_STANDARD=17"
    "-DCMAKE_BUILD_TYPE=Release"
    "-DOHM_BUILD_SHARED=ON"

    # The two GPU backends, off for the reasons in the header comment. OHM_FEATURE_OPENCL
    # defaults to ON wherever find_package(OpenCL) succeeds, so leaving it unset is not the
    # same as disabling it.
    "-DOHM_FEATURE_CUDA=OFF"
    "-DOHM_FEATURE_OPENCL=OFF"

    # The reason for the whole package: the 2.5D navigable-surface map.
    "-DOHM_FEATURE_HEIGHTMAP=ON"
    # Eigen drives the eigen-decomposition behind CovarianceVoxel/NDT, which is what fills
    # HeightmapVoxel's surface normals. ohm_mapping derives slope from neighbouring cell
    # heights instead, so this is not load-bearing -- but it is cheap and it is what makes
    # the normal fields meaningful if anything downstream ever wants them.
    "-DOHM_FEATURE_EIGEN=ON"
    # TBB. Not the ray mapper (that is serial regardless) but the voxel block compression
    # queue and the region walks do use it.
    "-DOHM_FEATURE_THREADS=ON"

    # Off: needs OpenGL + GLEW + glfw3 to rasterise a heightmap to an image file. An
    # offline visualisation aid, and it would drag a GL stack onto a headless rover.
    "-DOHM_FEATURE_HEIGHTMAP_IMAGE=OFF"
    # Off: PDAL is a large dependency and only the offline point cloud readers in slamio/
    # use it. Nothing in the ROS path reads a file.
    "-DOHM_FEATURE_PDAL=OFF"
    # Off: GTest, and the suite is not run here.
    "-DOHM_FEATURE_TEST=OFF"
    "-DOHM_BUILD_DOXYGEN=OFF"
  ];

  meta = {
    description = "OHM: probabilistic 3D occupancy mapping with a 2.5D heightmap layer (CPU-only build)";
    homepage = "https://github.com/csiro-robotics/ohm";
    license = with lib.licenses; [ mit ];
    platforms = lib.platforms.linux;
  };
}
