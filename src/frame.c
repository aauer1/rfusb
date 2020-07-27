/**
 * @file   frame.c
 * @date   30.04.2020
 * @author andreas
 * @brief  
 */

#include "frame.h"

//------------------------------------------------------------------------------
void frameInit(Frame *frame, Commands cmd, Flags flags)
{
    frame->command = cmd;
    frame->flags = flags;
    frame->length = 0;
    frame->crc = 0;
}

//------------------------------------------------------------------------------
void frameAddData(Frame *frame, const uint8_t *data, uint32_t size)
{
    for(uint32_t i=0; i<size; i++, frame->length++)
    {
        frame->data[frame->length] = data[i];
    }
}

