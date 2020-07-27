/**
 * @file   gpio.h
 * @date   21.12.2017
 * @author andreas
 * @brief  
 */

#ifndef GPIO_H_
#define GPIO_H_

#include <stm32l0xx.h>
#include <stm32l0xx_hal_gpio.h>

#define PORT_SIZE       16

void gpioSetup(uint8_t pin, const GPIO_InitTypeDef *init);
void gpioSetupExt(uint8_t pin, uint32_t mode, uint32_t pull, uint32_t speed, uint32_t alternate);
void gpioWrite(uint8_t pin, GPIO_PinState value);
void gpioSet(uint8_t pin);
void gpioClear(uint8_t pin);
void gpioToggle(uint8_t pin);
uint8_t gpioRead(uint8_t pin);

#endif /* GPIO_H_ */

