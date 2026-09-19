// board_id.cpp

#include "board_id.hpp"

#include "hardware/gpio.h"
#include "pico/time.h"

uint8_t read_board_id()
{
    gpio_init(kBoardIdPin0);
    gpio_set_dir(kBoardIdPin0, GPIO_IN);
    gpio_pull_up(kBoardIdPin0);

    gpio_init(kBoardIdPin1);
    gpio_set_dir(kBoardIdPin1, GPIO_IN);
    gpio_pull_up(kBoardIdPin1);

    gpio_init(kBoardIdPin2);
    gpio_set_dir(kBoardIdPin2, GPIO_IN);
    gpio_pull_up(kBoardIdPin2);

    sleep_us(10);  // let the pull-ups settle before reading

    // Switch on -> pin low -> bit set. See the comment in board_id.hpp.
    uint8_t id = 0;
    if (!gpio_get(kBoardIdPin0))
        id |= (1u << 0);
    if (!gpio_get(kBoardIdPin1))
        id |= (1u << 1);
    if (!gpio_get(kBoardIdPin2))
        id |= (1u << 2);
    return id;
}
