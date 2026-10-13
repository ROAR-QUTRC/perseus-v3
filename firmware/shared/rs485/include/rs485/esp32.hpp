// esp32.hpp
//
// ESP32 backend for rs485::Port. One hardware UART does both TX and RX, and
// the UART's RTS output drives the transceiver's DE/~RE pin through the
// driver's RS485 half-duplex mode. That matches the SBB carrier board
// (hardware/dc-motor-driver, sheets RS485_1 / RS485_2): each SP3485 has DE and
// ~RE tied together to a single DIR pin, so DIR is used as the UART's RTS.

#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "rs485/port.hpp"

namespace rs485
{
    class Esp32Port : public Port
    {
    public:
        // uart: UART_NUM_1 or UART_NUM_2 (UART_NUM_0 is the console).
        // dir_pin: wired to the transceiver's combined DE/~RE; high = transmit.
        Esp32Port(uart_port_t uart, gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t dir_pin,
                  uint32_t baud_hz);
        ~Esp32Port() override;

        Esp32Port(const Esp32Port&) = delete;
        Esp32Port& operator=(const Esp32Port&) = delete;

        // Installs and configures the UART driver. Call once at boot; returns
        // false if any step fails. send() and receive() do nothing until it
        // has succeeded.
        bool init();

        // Blocks until the last byte has left the wire and DE has been released.
        bool send(const uint8_t* data, size_t len) override;

        // Sleeps (the task is blocked, not spinning) until the first byte
        // arrives or first_byte_timeout_us passes, then keeps reading until
        // frame_gap_us of silence. The gap is timed in microseconds, so it is
        // not limited by the FreeRTOS tick: long gaps sleep tick by tick and
        // only the final stretch (under 2.5ms) is a short spin.
        size_t receive(uint8_t* buf, size_t cap, uint32_t first_byte_timeout_us,
                       uint32_t frame_gap_us) override;

        void flush_rx() override;
        uint32_t baud_hz() const override { return baud_hz_; }

    private:
        uart_port_t uart_;
        gpio_num_t tx_pin_;
        gpio_num_t rx_pin_;
        gpio_num_t dir_pin_;
        uint32_t baud_hz_;
        bool installed_ = false;
    };
}  // namespace rs485
