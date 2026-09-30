# This file was automatically generated for projects without default
# 'CMakeLists.txt' file.

file(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

# env:can_sim swaps main.cpp for a bench entry point that fakes the CAN bus.
if(CAN_SIM)
  list(FILTER app_sources EXCLUDE REGEX ".*/src/main\\.cpp$")
  list(APPEND app_sources ${CMAKE_SOURCE_DIR}/sim/can_sim.cpp)
endif()

idf_component_register(
  SRCS
  ${app_sources}
  REQUIRES
  board-support
  hi-can
  modbus-core
  rs485
  PRIV_REQUIRES
  esp_driver_sdm)

# Create and import the library targets
# add_subdirectory(../../../software/shared/hi-can hi-can)

# Publicly link `foo` to `main` component target_link_libraries(main PUBLIC
# hi-can)
