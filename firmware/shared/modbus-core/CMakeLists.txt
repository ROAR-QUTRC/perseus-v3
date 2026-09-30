cmake_minimum_required(VERSION 3.13)

# Platform-neutral Modbus RTU core. No SDK dependencies: it talks to the wire
# through rs485::Port (firmware/shared/rs485) and takes a modbus::Clock for
# time. Usable via add_subdirectory() from a Pico SDK project, or as a plain
# library for the host tests below.
project(modbus_core LANGUAGES CXX)

if(NOT TARGET rs485_port)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../rs485
                   ${CMAKE_CURRENT_BINARY_DIR}/rs485)
endif()

add_library(modbus_core STATIC src/protocol.cpp src/slave.cpp src/master.cpp)

target_include_directories(modbus_core PUBLIC include)
target_compile_features(modbus_core PUBLIC cxx_std_17)
target_link_libraries(modbus_core PUBLIC rs485_port)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(modbus_core PRIVATE -Wall -Wextra -fno-exceptions
                                             -fno-rtti)
endif()

option(MODBUS_CORE_BUILD_TESTS "Build the host-side unit tests" OFF)
if(MODBUS_CORE_BUILD_TESTS)
  enable_testing()
  add_executable(modbus_core_tests tests/test_modbus_core.cpp)
  target_link_libraries(modbus_core_tests PRIVATE modbus_core)
  target_compile_options(modbus_core_tests PRIVATE -Wall -Wextra)
  add_test(NAME modbus_core_tests COMMAND modbus_core_tests)
endif()
