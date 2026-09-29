cmake_minimum_required(VERSION 3.13)

# ====================================================================================
# Doom for the encoder board. The game runs on the RP2350; DOOM_PLATFORM picks how
# it is shown and controlled:
#   web  - the board is a USB network adapter serving a page with the game (now)
#   hdmi - HSTX DVI out of the USB-C port plus a USB-host keyboard (TBD)
set(TARGET_NAME doom)
set(TARGET_VERSION 0.1)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
# Same chip and board as the encoder firmware (see ../encoder/CMakeLists.txt).
set(PICO_BOARD
    none
    CACHE STRING "Board type")
set(PICO_PLATFORM
    rp2350-arm-s
    CACHE STRING "Platform")

include(pico_sdk_import.cmake)

project(${TARGET_NAME} CXX C ASM)
pico_sdk_init()

set(DOOM_PLATFORM
    web
    CACHE STRING "How the game is shown and controlled: web, or hdmi (TBD)")
set_property(CACHE DOOM_PLATFORM PROPERTY STRINGS web hdmi)

add_executable(
  ${TARGET_NAME}
  main.cpp
  engine/test_pattern.cpp # stand-in until the Doom engine lands
)
target_include_directories(${TARGET_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_compile_definitions(${TARGET_NAME} PRIVATE PICO_RP2350A=1 PICO_RP2350_A2_SUPPORTED=1)
pico_override_flash_size(${TARGET_NAME} "(16 * 1024 * 1024)")
target_link_libraries(${TARGET_NAME} pico_stdlib pico_multicore)

# ====================================================================================
if(DOOM_PLATFORM STREQUAL "web")
  set(WEB ${CMAKE_CURRENT_LIST_DIR}/platform/web)

  # The page lives in flash; rebuilt whenever index.html changes.
  set(INDEX_HTML_C ${CMAKE_CURRENT_BINARY_DIR}/index_html.c)
  add_custom_command(
    OUTPUT ${INDEX_HTML_C}
    COMMAND ${CMAKE_COMMAND} -DIN=${WEB}/index.html -DOUT=${INDEX_HTML_C} -DNAME=index_html -P
            ${CMAKE_CURRENT_LIST_DIR}/cmake/embed_file.cmake
    DEPENDS ${WEB}/index.html ${CMAKE_CURRENT_LIST_DIR}/cmake/embed_file.cmake
    COMMENT "Embedding index.html")

  target_sources(
    ${TARGET_NAME}
    PRIVATE ${WEB}/backend.cpp # core 1 loop, frame and key handoff between cores
            ${WEB}/net.cpp # TinyUSB NCM <-> lwIP, DHCP server
            ${WEB}/http_server.cpp # page, frame stream, key POSTs
            ${WEB}/png.cpp # palette PNG encoder
            ${WEB}/keymap.cpp # KeyboardEvent.code -> HID usage
            ${WEB}/usb_descriptors.c # CDC console + NCM network adapter
            ${PICO_TINYUSB_PATH}/lib/networking/dhserver.c
            ${INDEX_HTML_C})
  target_include_directories(${TARGET_NAME} PRIVATE ${WEB} ${PICO_TINYUSB_PATH}/lib/networking)
  target_link_libraries(${TARGET_NAME} tinyusb_device pico_lwip pico_lwip_arch pico_unique_id)

  # printf goes to the CDC console; the backend initialises it on core 1.
  pico_enable_stdio_usb(${TARGET_NAME} 1)
  pico_enable_stdio_uart(${TARGET_NAME} 0)
  target_compile_definitions(${TARGET_NAME} PRIVATE PICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1)
elseif(DOOM_PLATFORM STREQUAL "hdmi")
  message(FATAL_ERROR "DOOM_PLATFORM=hdmi is TBD (HSTX DVI + USB-host keyboard)")
else()
  message(FATAL_ERROR "Unknown DOOM_PLATFORM '${DOOM_PLATFORM}' (web or hdmi)")
endif()

pico_set_program_name(${TARGET_NAME} "${TARGET_NAME}")
pico_set_program_version(${TARGET_NAME} "${TARGET_VERSION}")
pico_add_extra_outputs(${TARGET_NAME})
