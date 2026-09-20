cmake_minimum_required(VERSION 3.13)

# Shared RS485 physical layer. rs485/port.hpp is the chip-neutral interface;
# each chip gets its own backend, built only when that chip's SDK is present.
#
# ESP32 (the SBB): src/esp32.cpp is not built from this file. ESP-IDF projects
# pick it up through the component wrapper at firmware/components/rs485.
project(rs485 LANGUAGES CXX)

add_library(rs485_port INTERFACE)
target_include_directories(rs485_port INTERFACE include)
target_compile_features(rs485_port INTERFACE cxx_std_17)

# RP2350: hardware UART RX + PIO TX. Needs pico_sdk_init() to have run.
if(TARGET pico_stdlib)
  add_library(rs485_rp2350 STATIC src/rp2350.cpp)
  target_link_libraries(
    rs485_rp2350
    PUBLIC rs485_port
    PRIVATE pico_stdlib hardware_uart hardware_pio)
  pico_generate_pio_header(rs485_rp2350
                           ${CMAKE_CURRENT_LIST_DIR}/src/uart_tx.pio)
endif()
