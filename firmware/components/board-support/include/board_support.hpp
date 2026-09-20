#pragma once

#include <driver/gpio.h>

#include <tuple>

/// @brief Board Support Package - contains board-specific constants and
/// configurations
namespace bsp
{
    /**
     * @brief An ordered pair of GPIO pins.
     *
     * When used for a CAN bus or UART, first pin is for TX, and the second is RX.
     * When used for a motor driver, first pin is for A, and the second is for B.
     */
    typedef std::pair<gpio_num_t, gpio_num_t> pin_pair_t;

    constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_0;
    constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_11;

    constexpr gpio_num_t I2C_SDA = GPIO_NUM_35;
    constexpr gpio_num_t I2C_SCL = GPIO_NUM_36;

    constexpr gpio_num_t A1 = GPIO_NUM_1;
    constexpr gpio_num_t A2 = GPIO_NUM_2;
    constexpr gpio_num_t A3 = GPIO_NUM_3;
    constexpr gpio_num_t A4 = GPIO_NUM_4;
    constexpr gpio_num_t A5 = GPIO_NUM_5;
    constexpr gpio_num_t A6 = GPIO_NUM_6;
    constexpr gpio_num_t A7 = GPIO_NUM_7;
    constexpr gpio_num_t A8 = GPIO_NUM_8;
    constexpr gpio_num_t A9 = GPIO_NUM_9;
    constexpr gpio_num_t A10 = GPIO_NUM_10;

    /**
     * @brief One RS485 bus: the UART's TX and RX pins plus the transceiver's
     * direction pin.
     */
    struct rs485_pins_t
    {
        gpio_num_t tx;
        gpio_num_t rx;
        gpio_num_t dir;  // drives the SP3485's DE and ~RE tied together: high = transmit
    };

    // RS485 buses on the dc-motor-driver carrier (hardware/dc-motor-driver,
    // sheets RS485_1 and RS485_2). Named after the schematic nets, whose SBB
    // pins are A7/A8/A9 (bus 1) and A10/IO12/IO39 (bus 2).
    constexpr rs485_pins_t RS485_1 = {A9, A8, A7};                     // MCU1_TX, MCU1_RX, MCU1_DIR
    constexpr rs485_pins_t RS485_2 = {GPIO_NUM_12, A10, GPIO_NUM_39};  // MCU2_TX, MCU2_RX, MCU2_DIR

#if __has_include(<Wire.h>)
    void initI2C();
#endif
}  // namespace bsp