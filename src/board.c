/**
 * @file   board.c
 * @date   19.09.2019
 * @author andreas
 * @brief  
 */

#include "board.h"

#include "gpio.h"

#include <stdint.h>

static const PinConfig BOARD_CONFIG[] =
{
        {
            .pin = 2,
            .init = { .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_LOW, .Alternate = 0 }
        },
        {
            .pin = 3,
            .init = { .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP, .Speed = GPIO_SPEED_HIGH, .Alternate = 0 }
        },
        {
            .pin = 4,
            .init = { .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_HIGH, .Alternate = 0 }
        },
        {
            .pin = 5,
            .init = { .Mode = GPIO_MODE_AF_PP, .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_HIGH, .Alternate = GPIO_AF0_SPI1 }
        },
        {
            .pin = 6,
            .init = { .Mode = GPIO_MODE_AF_PP, .Pull = GPIO_PULLUP, .Speed = GPIO_SPEED_HIGH, .Alternate = GPIO_AF0_SPI1 }
        },
        {
            .pin = 7,
            .init = { .Mode = GPIO_MODE_AF_PP, .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_HIGH, .Alternate = GPIO_AF0_SPI1 }
        },
        {
            .pin = 8,
            .init = { .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_HIGH, .Alternate = 0 }
        },
        {
            .pin = 9,
            .init = { .Mode = GPIO_MODE_AF_PP, .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_HIGH, .Alternate = GPIO_AF4_USART1 }
        },
        {
            .pin = 10,
            .init = { .Mode = GPIO_MODE_AF_PP, .Pull = GPIO_PULLUP, .Speed = GPIO_SPEED_HIGH, .Alternate = GPIO_AF4_USART1 }
        },
        {
            .pin = 14,
            .init = { .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_LOW, .Alternate = 0 }
        },
        {
            .pin = 15,
            .init = { .Mode = GPIO_MODE_INPUT, .Pull = GPIO_PULLUP, .Speed = GPIO_SPEED_HIGH, .Alternate = 0 }
        },
        {
            .pin = 16,
            .init = { .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_LOW, .Alternate = 0 }
        },
        {
            .pin = 17,
            .init = { .Mode = GPIO_MODE_OUTPUT_PP, .Pull = GPIO_NOPULL, .Speed = GPIO_SPEED_LOW, .Alternate = 0 }
        },
        {
            .pin = 255,
        }
};

//------------------------------------------------------------------------------
void boardInit(void)
{
    uint32_t i=0;

    for(i=0; BOARD_CONFIG[i].pin != 255; i++)
    {
        gpioSetup(BOARD_CONFIG[i].pin, &(BOARD_CONFIG[i].init));
    }
}
