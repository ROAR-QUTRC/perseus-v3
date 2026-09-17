# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

# ====================================================================================
# Project/target name. Renaming later should only ever require changing this line
# copying the file as a starting should also only require changing this line 
set(TARGET_NAME blink)
set(TARGET_VERSION 0.1)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # lets clangd/IDEs find compiler flags

# ====================================================================================
# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
  set(USERHOME $ENV{USERPROFILE})
else()
  set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if(EXISTS ${picoVscode})
  include(${picoVscode})
endif()

# ====================================================================================
# Chip/board selection.

# PICO_BOARD "none" tells the SDK there is no board header to pull defaults
# from - used for custom/bare RP2350 silicon rather than an official board
# Because of this, everything a board header would supply (platform, 
# chip varient, stepping, default pins) has to be given explicitly below
set(PICO_BOARD none CACHE STRING "Board type")

# RP2350 has two possible cores:
#    rp2350-arm-s -> Cortex-M33 (Arm), the default
#    rp2350-riscv -> Hazard3 (RISC-V)
set(PICO_PLATFORM rp2350-arm-s CACHE STRING "Platform")

# Pull in Raspberry Pi Pico SDK (must be before project())
include(pico_sdk_import.cmake)

project(${TARGET_NAME} CXX C ASM)
# C and ASM are kept alongside CXX because the SDK's own sources
# (and the startup/runtime code) are written in C and assembly;
# only the custom application code needs to be C++

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(${TARGET_NAME} ${TARGET_NAME}.cpp)
# If the app is split into multiple files later, add them here e.g.:
#   add_executable(${TARGET_NAME} ${TARGET_NAME}.cpp led.cpp sensors.cpp)

pico_set_program_name(${TARGET_NAME} "${TARGET_NAME}")
pico_set_program_version(${TARGET_NAME} "${TARGET_VERSION}")

# ====================================================================================
# Chip-variant / silicon-revision definitions.
#
# A board header would normally provide these; with PICO_BOARD=none they must
# be set here instead. Targeting RP2350A, revision A2:
#   PICO_RP2350A            1 -> RP2350A package/die (30 user GPIOs, QFN-60).
#                                Set to 0 if using RP2350B
#                                (48 GPIOs, more ADC/PWM channels).
#   PICO_RP2350_A2_SUPPORTED 1 -> silicon revision A2, the current production
#                                stepping. Leave this set unless specifically
#                                using older A1 silicon.
#   NEOPIXEL_PIN            -> GPIO wired to the WS2812/NeoPixel's data-in
#                                pin (this is the compile-time constant,
#                                not an SDK-defined one like the two above).

target_compile_definitions(${TARGET_NAME} PRIVATE 
  PICO_RP2350A=1
  PICO_RP2350_A2_SUPPORTED=1
  NEOPIXEL_PIN=4)

# ====================================================================================
# Compile ws2812.pio (the PIO assembly that generates the WS2812 signal)
# into ws2812.pio.h, available to blink.cpp via #include "ws2812.pio.h".
# Any future .pio files get added here the same way.
pico_generate_pio_header(${TARGET_NAME} ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

# ====================================================================================
# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(${TARGET_NAME} 0)
pico_enable_stdio_usb(${TARGET_NAME} 0)

# ====================================================================================
# Linked libraries. pico_stdlib is the baseline (GPIO, timing, printf, etc.)
# Add more SDK components as functionality grows, e.g.:
#   target_link_libraries(${TARGET_NAME}
#      pico_stdlib
#      hardware_i2c      # I2C peripherals
#      hardware_spi      # SPI peripherals
#      hardware_pwm      # PWM outputs
#      hardware_adc      # analog inputs
#      hardware_pio      # PIO system
#      pico_multicore    # run code on the second core
#   )
# Note: pico_cyw43_arch_* (wireless) libraries are not included.
# They are specific to boards with CyW43 radio, which the encoders do not include

target_link_libraries(${TARGET_NAME}
  pico_stdlib
  hardware_pio  # needed to drive ws2812.pio's state machine
)

# create map/bin/hex file etc.
pico_add_extra_outputs(${TARGET_NAME})
