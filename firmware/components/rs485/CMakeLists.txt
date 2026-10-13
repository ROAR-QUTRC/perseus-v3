# ESP-IDF build of the shared RS485 library (firmware/shared/rs485): the
# chip-neutral rs485::Port interface plus the ESP32 backend. The RP2350 backend
# in the same library is built by the Pico project, not here.
idf_component_register(
  SRCS
  ../../shared/rs485/src/esp32.cpp
  INCLUDE_DIRS
  ../../shared/rs485/include
  REQUIRES
  driver
  PRIV_REQUIRES
  esp_timer
  esp_rom)
