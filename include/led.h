/**
 * @file   led.h
 * @date   28.02.2016
 * @author andreas
 * @brief  
 */

#ifndef LED_H_
#define LED_H_

#include "gpio.h"

#define LED0            16
#define LED1            17

#define ledEnable(led)      gpioSet(led)
#define ledDisable(led)     gpioClear(led)
#define ledToggle(led)      gpioToggle(led)


#endif /* LED_H_ */
