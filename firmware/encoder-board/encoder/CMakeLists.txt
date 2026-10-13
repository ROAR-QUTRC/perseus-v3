# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

# ====================================================================================
# Target name. Renaming, or copying this file as a starting point, only needs this line changed.
set(TARGET_NAME encoder)
set(TARGET_VERSION 0.1)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # lets clangd/IDEs find compiler flags

# ====================================================================================
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
# Chip/board selection. PICO_BOARD "none" = custom RP2350 board, so platform,
# chip variant and stepping are given explicitly below.
set(PICO_BOARD
    none
    CACHE STRING "Board type")

# rp2350-arm-s (Cortex-M33) or rp2350-riscv (Hazard3)
set(PICO_PLATFORM
    rp2350-arm-s
    CACHE STRING "Platform")

# Pull in Raspberry Pi Pico SDK (must be before project())
include(pico_sdk_import.cmake)

project(${TARGET_NAME} CXX C ASM)
# C and ASM stay enabled for the SDK's own sources.

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# ====================================================================================
# FreeRTOS-Kernel isn't bundled with pico-sdk. The path is set explicitly
# because FreeRTOS_Kernel_import.cmake's sibling auto-detection misses the
# Community-Supported-Ports layout, where the RP2350 ARM SMP port lives.
set(FREERTOS_KERNEL_PATH
    "${USERHOME}/.pico-sdk/sdk/FreeRTOS-Kernel"
    CACHE PATH "Path to the FreeRTOS Kernel")
include(FreeRTOS_Kernel_import.cmake)

# Layout: drivers/ = hardware wrappers (no FreeRTOS); rtos/ = tasks, shared
# state and FreeRTOSConfig.h; encoder.cpp = board bring-up. RS485 and Modbus live
# in the shared libraries linked below (firmware/shared/rs485, modbus-core).
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../shared/rs485
                 ${CMAKE_BINARY_DIR}/rs485)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../shared/modbus-core
                 ${CMAKE_BINARY_DIR}/modbus-core)

add_executable(
  ${TARGET_NAME}
  ${TARGET_NAME}.cpp
  drivers/as5600.cpp # AS5600 magnetic rotary encoder driver (I2C)
  drivers/status_led.cpp # WS2812 status LED state machine (heartbeat/velocity)
  drivers/ws2812.cpp # drives ws2812.pio for the status LED
  rtos/encoder_task.cpp # core 1: owns the AS5600
  rtos/comms_task.cpp # core 0: RS485/Modbus RTU slave
  rtos/led_task.cpp # core 0: status LED
)

# drivers/ and rtos/ are both include roots, so headers are included by bare filename.
target_include_directories(${TARGET_NAME} PRIVATE drivers rtos)

# ====================================================================================
# Compile ws2812.pio into ws2812.pio.h (included by drivers/ws2812.cpp).
pico_generate_pio_header(${TARGET_NAME}
                         ${CMAKE_CURRENT_LIST_DIR}/drivers/ws2812.pio)

pico_set_program_name(${TARGET_NAME} "${TARGET_NAME}")
pico_set_program_version(${TARGET_NAME} "${TARGET_VERSION}")

# ====================================================================================
# Chip variant and pins (no board header with PICO_BOARD=none).
#   PICO_RP2350A, PICO_RP2350_A2_SUPPORTED: RP2350A package, silicon revision A2.
#   AS5600_SDA_PIN / SCL_PIN: I2C1 pin pair. AS5600_DIR_PIN: rotation strap.
#   NEOPIXEL_PIN: status LED data-in.
#   configNUMBER_OF_CORES: FreeRTOS SMP; encoder_task gets core 1, comms_task
#     and led_task share core 0.

target_compile_definitions(
  ${TARGET_NAME}
  PRIVATE PICO_RP2350A=1
          PICO_RP2350_A2_SUPPORTED=1
          AS5600_SDA_PIN=2
          AS5600_SCL_PIN=3
          AS5600_DIR_PIN=8
          NEOPIXEL_PIN=4
          configNUMBER_OF_CORES=2
          # Without this, stdio_usb only transmits once the host asserts DTR,
          # which some serial monitors don't do.
          PICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1)

# ====================================================================================
# USB stdio is on so angle readings can be watched over the board's USB connection.
pico_enable_stdio_uart(${TARGET_NAME} 0)
pico_enable_stdio_usb(${TARGET_NAME} 1)

# ====================================================================================
# Linked libraries
target_link_libraries(
  ${TARGET_NAME}
  pico_stdlib
  hardware_i2c # talks to the AS5600 over I2C
  hardware_pio # drives ws2812.pio for the status LED
  rs485_rp2350 # RS485 physical layer (firmware/shared/rs485)
  modbus_core # shared Modbus RTU core (firmware/shared/modbus-core)
  FreeRTOS-Kernel-Heap4 # SMP kernel; the RP2350 port brings up core 1 itself
)

# create map/bin/hex file etc.
pico_add_extra_outputs(${TARGET_NAME})
