#include <stdint.h>

#include "stm32l0xx.h"
#include "clock.h"
#include "led.h"

//------------------------------------------------------------------------------
void SysTick_Handler(void)
{
    HAL_IncTick();
}

//------------------------------------------------------------------------------
void clockInit(void)
{
    HAL_InitTick(0);
}

//------------------------------------------------------------------------------
uint64_t clockGetTime(void)
{
	return HAL_GetTick();
}
