/**
 * @file   board.h
 * @date   20.12.2017
 * @author andreas
 * @brief  
 */

#ifndef BOARD_H_
#define BOARD_H_

#include <stm32l0xx.h>

typedef struct PinConfig_ PinConfig;

struct PinConfig_
{
    uint8_t pin;
    GPIO_InitTypeDef init;
};

void boardInit(void);

#endif /* BOARD_H_ */
