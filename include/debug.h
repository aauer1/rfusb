/**
 * @file   debug.h
 * @date   28.02.2016
 * @author andreas
 * @brief  
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include <stdio.h>

#include "stm32l0xx_hal.h"

#define LEVEL_DEBUG         0
#define LEVEL_INFO          1

#define debug(format, ...)      debugWrite(LEVEL_DEBUG, __func__, format "\r\n", ##__VA_ARGS__)
#define info(format, ...)       debugWrite(LEVEL_INFO, __func__, format "\r\n", ##__VA_ARGS__)

void debugInit(void);
void debugWrite(uint8_t level, const char *name, const char *format, ...);

#endif /* DEBUG_H_ */
