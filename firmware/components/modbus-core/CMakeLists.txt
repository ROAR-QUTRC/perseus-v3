# ESP-IDF build of the shared Modbus core (firmware/shared/modbus-core), plus
# the ESP32 glue that lives here (modbus/esp32_clock.hpp), the same split as
# the hi-can component: chip-neutral sources stay in shared/, platform glue
# stays in components/.
idf_component_register(
  SRCS
  ../../shared/modbus-core/src/protocol.cpp
  ../../shared/modbus-core/src/slave.cpp
  ../../shared/modbus-core/src/master.cpp
  INCLUDE_DIRS
  ../../shared/modbus-core/include
  ./include
  REQUIRES
  rs485
  esp_timer
  esp_rom)
