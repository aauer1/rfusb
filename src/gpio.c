/**
 * @file   gpio.c
 * @date   21.12.2017
 * @author andreas
 * @brief  
 */

#include "gpio.h"

#include <string.h>

//------------------------------------------------------------------------------
static GPIO_TypeDef *getGpio(uint8_t pin)
{
    GPIO_TypeDef *ret = 0;
    uint8_t port = pin / PORT_SIZE;

    switch(port)
    {
        case 0: ret = GPIOA; break;
        case 1: ret = GPIOB; break;
        case 2: ret = GPIOC; break;
        case 3: ret = GPIOD; break;
        case 4: ret = GPIOE; break;
        case 7: ret = GPIOH; break;
        default:
            ret = 0;
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
void gpioSetup(uint8_t pin, const GPIO_InitTypeDef *init)
{
    GPIO_TypeDef *port = getGpio(pin);
    uint16_t mask = pin % PORT_SIZE;
    GPIO_InitTypeDef temp;

    memcpy(&temp, init, sizeof(GPIO_InitTypeDef));

    mask = (1 << mask);
    temp.Pin = mask;

    HAL_GPIO_Init(port, &temp);
}

//------------------------------------------------------------------------------
void gpioSetupExt(uint8_t pin, uint32_t mode, uint32_t pull, uint32_t speed, uint32_t alternate)
{
    GPIO_InitTypeDef init;

    init.Mode = mode;
    init.Pull = pull;
    init.Speed = speed;
    init.Alternate = alternate;

    gpioSetup(pin, &init);
}

//------------------------------------------------------------------------------
void gpioWrite(uint8_t pin, GPIO_PinState value)
{
    GPIO_TypeDef *port = getGpio(pin);
    uint16_t mask = pin % PORT_SIZE;

    mask = (1 << mask);

    HAL_GPIO_WritePin(port, mask, value);
}

//------------------------------------------------------------------------------
void gpioSet(uint8_t pin)
{
    GPIO_TypeDef *port = getGpio(pin);
    uint16_t mask = pin % PORT_SIZE;

    mask = (1 << mask);

    HAL_GPIO_WritePin(port, mask, GPIO_PIN_SET);
}

//------------------------------------------------------------------------------
void gpioClear(uint8_t pin)
{
    GPIO_TypeDef *port = getGpio(pin);
    uint16_t mask = pin % PORT_SIZE;

    mask = (1 << mask);

    HAL_GPIO_WritePin(port, mask, GPIO_PIN_RESET);
}

//------------------------------------------------------------------------------
void gpioToggle(uint8_t pin)
{
    GPIO_TypeDef *port = getGpio(pin);
    uint16_t mask = pin % PORT_SIZE;

    mask = (1 << mask);

    HAL_GPIO_TogglePin(port, mask);
}

//------------------------------------------------------------------------------
uint8_t gpioRead(uint8_t pin)
{
    GPIO_TypeDef *port = getGpio(pin);
    uint16_t mask = pin % PORT_SIZE;

    mask = (1 << mask);

    return (HAL_GPIO_ReadPin(port, mask) == GPIO_PIN_SET);
}

