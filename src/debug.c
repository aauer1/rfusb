/**
 * @file   debug.c
 * @date   28.02.2016
 * @author andreas
 * @brief  
 */

#include "debug.h"
#include "config/config.h"

#include "stm32l0xx.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

static USART_HandleTypeDef usart_;

static char buffer[256];

//------------------------------------------------------------------------------
static HAL_StatusTypeDef serialWrite(char *data, uint32_t len)
{
    return HAL_USART_Transmit(&usart_, (uint8_t *)data, len, 1000);
}

//------------------------------------------------------------------------------
void debugInit(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    usart_.Instance = USART1;
    usart_.Init.BaudRate = 115200;
    usart_.Init.Mode = USART_MODE_TX;
    usart_.Init.Parity = USART_PARITY_NONE;
    usart_.Init.StopBits = USART_STOPBITS_1;
    usart_.Init.WordLength = USART_WORDLENGTH_8B;
    usart_.Init.CLKPolarity = USART_POLARITY_LOW;
    usart_.Init.CLKPhase = USART_PHASE_1EDGE;
    usart_.Init.CLKLastBit = USART_LASTBIT_DISABLE;

    HAL_USART_Init(&usart_);
}

//------------------------------------------------------------------------------
void debugWrite(uint8_t level, const char *name, const char *format, ...)
{
    va_list args;
    int len;

    if(level < DEBUG_LEVEL)
    {
        return;
    }

    switch(level)
    {
        case LEVEL_DEBUG:
            len = snprintf(buffer, sizeof(buffer), "\x1b[0m[%6lu] [%16s]  ", HAL_GetTick(), name);
            break;

        case LEVEL_INFO:
            len = snprintf(buffer, sizeof(buffer), "\x1b[36m[%6lu] [%16s]  ", HAL_GetTick(), name);
            break;

        default:
            len = 0;
            break;
    }

    if(len >= 0)
    {
        va_start(args, format);
        len += vsnprintf(buffer + len, sizeof(buffer) - len, format, args);
        va_end (args);
    }
    else
    {
        len = sprintf(buffer, "\x1b[91mSome logging error occured\r\n");
    }

    if(len > 0)
    {
        serialWrite(buffer, len);
        serialWrite("\x1b[0m", 4);
    }
}
