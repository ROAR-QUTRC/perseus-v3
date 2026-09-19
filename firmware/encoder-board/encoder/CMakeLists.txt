# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

# ====================================================================================
# Project/target name. Renaming later should only ever require changing this line
# copying the file as a starting should also only require changing this line
set(TARGET_NAME encoder)
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
# chip variant, stepping, default pins) has to be given explicitly below
set(PICO_BOARD
    none
    CACHE STRING "Board type")

# RP2350 has two possible cores:
#    rp2350-arm-s -> Cortex-M33 (Arm), the default
#    rp2350-riscv -> Hazard3 (RISC-V)
set(PICO_PLATFORM
    rp2350-arm-s
    CACHE STRING "Platform")

# Pull in Raspberry Pi Pico SDK (must be before project())
include(pico_sdk_import.cmake)

project(${TARGET_NAME} CXX C ASM)
# C and ASM are kept alongside CXX because the SDK's own sources
# (and the startup/runtime code) are written in C and assembly;
# only the custom application code needs to be C++

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# ====================================================================================
# FreeRTOS-Kernel. Not bundled with pico-sdk -- expects FREERTOS_KERNEL_PATH
# (env var or CMake var) to point at a FreeRTOS-Kernel checkout containing
# the RP2350 ARM SMP port. On this dev machine that's a sibling of
# PICO_SDK_PATH (~/.pico-sdk/sdk/FreeRTOS-Kernel). Set explicitly here
# rather than relying on FreeRTOS_Kernel_import.cmake's own sibling
# auto-detection: that logic locks in whichever of its two port-tree
# layouts it happens to be checking when it finds the sibling directory
# (the old "portable/ThirdParty/GCC/<port>" layout) and never retries the
# other one (the current "portable/ThirdParty/Community-Supported-Ports"
# layout, where the RP2350 ARM port actually lives) -- so it fails to find
# a real, present port. Pre-setting the path here makes it run its proper
# two-pass check against a known-good directory instead of guessing at one.
set(FREERTOS_KERNEL_PATH "${USERHOME}/.pico-sdk/sdk/FreeRTOS-Kernel" CACHE PATH "Path to the FreeRTOS Kernel")
include(FreeRTOS_Kernel_import.cmake)

# Project layout:
#   drivers/  -- hardware wrappers, no FreeRTOS dependency (AS5600, RS485
#                transport, Modbus RTU framing, status LED state machine,
#                WS2812 PIO driver, board-id DIP-switch read). Each of
#                these could be pulled into a non-RTOS build unchanged.
#   rtos/     -- FreeRTOS-specific: the three tasks, the cross-core
#                SharedState/queue types, FreeRTOSConfig.h.
#   encoder.cpp (top level) -- application entry point: one-time board
#                bring-up, then creates the rtos/ tasks, which call into
#                drivers/ from there.
add_executable(
  ${TARGET_NAME}
  ${TARGET_NAME}.cpp
  drivers/as5600.cpp # AS5600 magnetic rotary encoder driver (I2C)
  drivers/board_id.cpp # reads the DIP-switch device ID
  drivers/rs485_transport.cpp # RS485 half-duplex physical layer (UART + DE/RE)
  drivers/modbus_rtu.cpp # Modbus RTU framing on top of rs485_transport
  drivers/status_led.cpp # WS2812 status LED state machine (heartbeat/velocity)
  drivers/ws2812.cpp # drives ws2812.pio for the status LED
  rtos/encoder_task.cpp # core 1: owns the AS5600
  rtos/comms_task.cpp # core 0: RS485/Modbus RTU slave
  rtos/led_task.cpp # core 0: status LED (higher priority than comms_task -- see rtos/led_task.hpp)
)

# Both directories are on the include path so #include "as5600.hpp" etc.
# stay bare filenames throughout, regardless of which side of the
# drivers/rtos split is including what.
target_include_directories(${TARGET_NAME} PRIVATE drivers rtos)

# ====================================================================================
# Compile ws2812.pio (the PIO assembly that generates the WS2812 signal)
# into ws2812.pio.h, available to drivers/ws2812.cpp via #include "ws2812.pio.h".
pico_generate_pio_header(${TARGET_NAME} ${CMAKE_CURRENT_LIST_DIR}/drivers/ws2812.pio)

# Compile uart_tx.pio (bit-banged UART TX for the RS485 link, see the class
# comment in rs485_transport.hpp for why) into uart_tx.pio.h.
pico_generate_pio_header(${TARGET_NAME} ${CMAKE_CURRENT_LIST_DIR}/drivers/uart_tx.pio)

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
#   AS5600_SDA_PIN           -> GPIO wired to the AS5600's SDA line.
#   AS5600_SCL_PIN           -> GPIO wired to the AS5600's SCL line. GPIO2/3
#                                is the RP2350's I2C1 pin pair.
#   AS5600_DIR_PIN           -> GPIO wired to the AS5600's DIR line (sets
#                                its rotation-direction convention); has a
#                                light external pull-up on the board.
#   NEOPIXEL_PIN             -> GPIO wired to the status LED's data-in line.
#   configNUMBER_OF_CORES    -> FreeRTOS SMP: encoder_task gets core 1 to
#                                itself, comms_task/led_task share core 0.
#                                See FreeRTOSConfig.h.

target_compile_definitions(
  ${TARGET_NAME}
  PRIVATE PICO_RP2350A=1
          PICO_RP2350_A2_SUPPORTED=1
          AS5600_SDA_PIN=2
          AS5600_SCL_PIN=3
          AS5600_DIR_PIN=8
          NEOPIXEL_PIN=4
          configNUMBER_OF_CORES=2
          # Without this, stdio_usb only transmits once the host
          # asserts DTR on the CDC port (some serial monitors,
          # e.g. vscode-serial-monitor on Linux, don't reliably
          # do this) -- output would otherwise be silently
          # dropped even though the board is running fine.
          PICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1)

# ====================================================================================
# Modify the below lines to enable/disable output over UART/USB
# USB stdio is on so angle readings can be observed with a serial monitor
# over the board's USB connection.
pico_enable_stdio_uart(${TARGET_NAME} 0)
pico_enable_stdio_usb(${TARGET_NAME} 1)

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

target_link_libraries(
  ${TARGET_NAME} pico_stdlib hardware_i2c # talks to the AS5600 over I2C
  hardware_uart # RS485/Modbus link (rs485_transport.cpp)
  hardware_pio # drives ws2812.pio for the status LED
  FreeRTOS-Kernel-Heap4 # SMP kernel; the RP2350 port brings up core 1 itself
)

# create map/bin/hex file etc.
pico_add_extra_outputs(${TARGET_NAME})
