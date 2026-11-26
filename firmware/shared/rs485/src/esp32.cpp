// esp32.cpp

#include "rs485/esp32.hpp"

#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace rs485
{
    namespace
    {
        // Bigger than the UART's 128-byte hardware FIFO, as the driver requires.
        constexpr int kRxBufferBytes = 256;

        // The UART hands received bytes to the driver's ring buffer once this
        // many character times pass with the line idle. Kept below Modbus's
        // 3.5-character frame gap so a whole frame is available promptly.
        constexpr uint8_t kRxTimeoutSymbols = 2;

        constexpr TickType_t kTxDoneTimeout = pdMS_TO_TICKS(200);

        // Round up, plus one tick: a wait of N ticks can expire after as little
        // as N-1 ticks' worth of real time, and a timeout must never be short.
        TickType_t ticks_at_least_us(uint32_t us)
        {
            return pdMS_TO_TICKS((us + 999u) / 1000u) + 1;
        }
    }  // namespace

    Esp32Port::Esp32Port(uart_port_t uart, gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t dir_pin,
                         uint32_t baud_hz)
        : uart_(uart),
          tx_pin_(tx_pin),
          rx_pin_(rx_pin),
          dir_pin_(dir_pin),
          baud_hz_(baud_hz)
    {
    }

    Esp32Port::~Esp32Port()
    {
        if (installed_)
            uart_driver_delete(uart_);
    }

    bool Esp32Port::init()
    {
        if (installed_)
            return true;

        uart_config_t config = {};
        config.baud_rate = static_cast<int>(baud_hz_);
        config.data_bits = UART_DATA_8_BITS;
        config.parity = UART_PARITY_DISABLE;
        config.stop_bits = UART_STOP_BITS_1;
        config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        config.source_clk = UART_SCLK_DEFAULT;

        if (uart_driver_install(uart_, kRxBufferBytes, 0, 0, nullptr, 0) != ESP_OK)
            return false;
        installed_ = true;

        if (uart_param_config(uart_, &config) != ESP_OK ||
            uart_set_pin(uart_, tx_pin_, rx_pin_, dir_pin_, UART_PIN_NO_CHANGE) != ESP_OK ||
            uart_set_mode(uart_, UART_MODE_RS485_HALF_DUPLEX) != ESP_OK ||
            uart_set_rx_timeout(uart_, kRxTimeoutSymbols) != ESP_OK)
        {
            uart_driver_delete(uart_);
            installed_ = false;
            return false;
        }
        return true;
    }

    bool Esp32Port::send(const uint8_t* data, size_t len)
    {
        if (!installed_)
            return false;
        if (len == 0)
            return true;

        if (uart_write_bytes(uart_, data, len) != static_cast<int>(len))
            return false;
        return uart_wait_tx_done(uart_, kTxDoneTimeout) == ESP_OK;
    }

    size_t Esp32Port::receive(uint8_t* buf, size_t cap, uint32_t first_byte_timeout_us,
                              uint32_t frame_gap_us)
    {
        if (!installed_ || cap == 0)
            return 0;

        // Sleep until something arrives.
        if (uart_read_bytes(uart_, buf, 1, ticks_at_least_us(first_byte_timeout_us)) <= 0)
            return 0;
        size_t count = 1;

        // Then gather the rest of the frame: the frame ends after frame_gap_us with
        // no new bytes.
        int64_t deadline = esp_timer_get_time() + frame_gap_us;
        while (count < cap)
        {
            size_t available = 0;
            uart_get_buffered_data_len(uart_, &available);
            if (available > 0)
            {
                const size_t take = available < (cap - count) ? available : (cap - count);
                const int got = uart_read_bytes(uart_, buf + count, take, 0);
                if (got > 0)
                {
                    count += static_cast<size_t>(got);
                    deadline = esp_timer_get_time() + frame_gap_us;
                }
                continue;
            }
            const int64_t remaining_us = deadline - esp_timer_get_time();
            if (remaining_us <= 0)
                break;
            if (remaining_us >= 2500)
                vTaskDelay(1);  // long gap (low baud): sleep, don't spin
            else
                esp_rom_delay_us(20);
        }
        return count;
    }

    void Esp32Port::flush_rx()
    {
        if (installed_)
            uart_flush_input(uart_);
    }
}  // namespace rs485
