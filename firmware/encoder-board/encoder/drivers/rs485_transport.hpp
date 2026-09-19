// rs485_transport.hpp
//
// Half-duplex RS485 physical layer: owns direction (DE/RE) control and
// byte-level TX/RX. This is the layer Modbus RTU framing (see
// modbus_rtu.hpp) sits on top of.

#pragma once

#include <cstddef>
#include <cstdint>

#include "hardware/pio.h"
#include "hardware/uart.h"

// Board-specific pin note (enc-encoder rev, see hardware/enc-encoder):
//   RS485_RX  -> GPIO5 (UART1 RX -- standard hardware pin mux, works as-is)
//   RS485_DIR -> GPIO6 (plain GPIO; drives the transceiver's combined
//                       DE/~RE, tied together on U4: high = transmit,
//                       low = receive)
//   RS485_TX  -> GPIO7 -- GPIO7 has NO hardware UART1 TX mux option on
//                       RP2350 (only uart1_rts/uart1_rx). GPIO4, the
//                       "normal" UART1 TX pin, is used for the WS2812
//                       status LED instead. TX is therefore bit-banged
//                       via a PIO UART (uart_tx.pio) rather than the
//                       hardware UART peripheral.
class Rs485Transport
{
public:
    // uart: hardware UART instance for RX (uart1 on this board).
    // rx_pin, dir_pin: as documented above.
    // tx_pio/tx_sm: PIO block + state machine dedicated to TX (uart_tx.pio).
    //   Uses a separate PIO block (pio1) from the status LED (pio0) so the
    //   two don't compete for program space or state machines.
    // tx_pin: RS485_TX (GPIO7).
    Rs485Transport(uart_inst_t* uart, uint rx_pin, uint dir_pin, PIO tx_pio, uint tx_sm,
                   uint tx_pin, uint32_t baud_hz);

    // Configures the hardware UART (RX), the PIO TX program, and the
    // direction pin (defaulting to receive). Call once at boot.
    void init();

    // Blocking send of a whole frame: raises DE/RE, shifts every byte out
    // via the PIO UART, waits for the line to actually finish (not just
    // for the FIFO to drain -- see the .cpp), then drops back to receive.
    bool send(const uint8_t* data, size_t len);

    // Reads up to `len` bytes, blocking until `timeout_ms` elapses with no
    // further bytes arriving (typical Modbus RTU frame-gap detection).
    // Returns the number of bytes read.
    //
    // Note: this busy-waits on the UART for up to timeout_ms even when the
    // bus is completely idle, since the gap timer starts as soon as the
    // call is made rather than only once a first byte arrives. Fine for a
    // bare-metal super-loop; once this runs as a FreeRTOS task, it should
    // become interrupt/DMA-driven instead so it can block cooperatively.
    size_t receive(uint8_t* buf, size_t len, uint32_t timeout_ms);

    uint32_t baud_hz() const { return baud_hz_; }

private:
    void set_driving(bool driving);

    uart_inst_t* uart_;
    uint rx_pin_;
    uint dir_pin_;
    PIO tx_pio_;
    uint tx_sm_;
    uint tx_pin_;
    uint32_t baud_hz_;
    uint32_t byte_period_us_;  // time to shift one 8N1 byte at baud_hz_, plus margin
};
