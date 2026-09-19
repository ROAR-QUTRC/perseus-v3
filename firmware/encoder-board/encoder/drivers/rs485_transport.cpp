// rs485_transport.cpp

#include "rs485_transport.hpp"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include "uart_tx.pio.h"  // generated from uart_tx.pio by pico_generate_pio_header()

Rs485Transport::Rs485Transport(uart_inst_t* uart, uint rx_pin, uint dir_pin, PIO tx_pio,
                               uint tx_sm, uint tx_pin, uint32_t baud_hz)
    : uart_(uart),
      rx_pin_(rx_pin),
      dir_pin_(dir_pin),
      tx_pio_(tx_pio),
      tx_sm_(tx_sm),
      tx_pin_(tx_pin),
      baud_hz_(baud_hz),
      // 10 bit periods/byte at 8N1 (start + 8 data + stop), plus one extra
      // bit period of margin.
      byte_period_us_(static_cast<uint32_t>(11ull * 1'000'000ull / baud_hz))
{
}

void Rs485Transport::init()
{
    uart_init(uart_, baud_hz_);
    gpio_set_function(rx_pin_, GPIO_FUNC_UART);

    uint offset = pio_add_program(tx_pio_, &uart_tx_program);
    uart_tx_program_init(tx_pio_, tx_sm_, offset, tx_pin_, baud_hz_);

    gpio_init(dir_pin_);
    gpio_set_dir(dir_pin_, GPIO_OUT);
    set_driving(false);
}

void Rs485Transport::set_driving(bool driving) { gpio_put(dir_pin_, driving); }

bool Rs485Transport::send(const uint8_t* data, size_t len)
{
    if (len == 0)
        return true;

    set_driving(true);
    // Give the transceiver a moment to actually switch from receive to
    // drive before the first start bit reaches it.
    sleep_us(10);

    for (size_t i = 0; i < len; ++i)
        pio_sm_put_blocking(tx_pio_, tx_sm_, data[i]);

    // pio_sm_put_blocking only guarantees each byte reached the TX FIFO,
    // not that it's finished shifting onto the wire -- wait for the FIFO
    // to empty, then one more byte period of margin so the last byte (and
    // its stop bit) actually finishes transmitting before the bus is
    // released back to receive.
    while (!pio_sm_is_tx_fifo_empty(tx_pio_, tx_sm_))
        tight_loop_contents();
    sleep_us(byte_period_us_);

    set_driving(false);
    return true;
}

size_t Rs485Transport::receive(uint8_t* buf, size_t len, uint32_t timeout_ms)
{
    size_t count = 0;
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);

    while (count < len && !time_reached(deadline))
    {
        if (uart_is_readable(uart_))
        {
            buf[count++] = uart_getc(uart_);
            deadline = make_timeout_time_ms(timeout_ms);  // reset gap timer on each byte
        }
    }
    return count;
}
