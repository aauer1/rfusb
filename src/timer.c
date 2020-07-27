/*
 * timer.c
 *
 *  Created on: 03.11.2011
 *      Author: andreas
 */

#include "timer.h"

#include <stm32l0xx.h>

//------------------------------------------------------------------------------
void timerSet(Timer *t, uint64_t interval)
{
    t->active = 1;
    t->start = HAL_GetTick();
    t->interval = interval;
}

//------------------------------------------------------------------------------
void timerStop(Timer *t)
{
    t->active = 0;
}

//------------------------------------------------------------------------------
void timerReset(Timer *t)
{
    t->active = 1;
    t->start += t->interval;
}

//------------------------------------------------------------------------------
void timerRestart(Timer *t)
{
    t->active = 1;
    t->start = HAL_GetTick();
}

//------------------------------------------------------------------------------
unsigned char timerExpired(Timer *t)
{
    uint64_t time = HAL_GetTick();

    if(!t->active)
    {
        return 0;
    }

    if(time > t->start)
    {
        uint64_t diff = (time - t->start);
        if(diff > t->interval)
        {
            t->active = 0;
            return 1;
        }
        else
        {
            return 0;
        }
    }

    return 0;
}
