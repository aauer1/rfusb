/**
 * @file   protocol.c
 * @date   23.04.2020
 * @author andreas
 * @brief  
 */

#include "protocol.h"
#include "usb/webusb.h"

#include "debug.h"

#include <string.h>

//------------------------------------------------------------------------------
void protocolInit(Protocol *proto)
{
    proto->onFrameReceived = 0;
    proto->send = 0;
    proto->pwrite = 0;
}

//------------------------------------------------------------------------------
void protocolSetCallback(Protocol *proto, void (*cb)(Protocol *, UsbFrame *))
{
    proto->onFrameReceived = cb;
}

//------------------------------------------------------------------------------
void protocolSetSendCallback(Protocol *proto, void (*cb)(uint8_t *, uint32_t))
{
    proto->send = cb;
}

//------------------------------------------------------------------------------
void protocolAddData(Protocol *proto, uint8_t *data, uint16_t size)
{
    uint16_t remaining = PROTOCOL_MAX_FRAME_SIZE - proto->pwrite;
    uint16_t offset = 0;

    if(proto->pwrite == 0)
    {
        for(offset = 0; offset < size; offset++)
        {
            if(data[offset] > CMD_INVALID && data[offset] < CMD_END)
            {
                break;
            }
        }
    }

    data += offset;
    size -= offset;
    if(remaining >= size)
    {
        memcpy(proto->rx_buffer + proto->pwrite, data, size);
        proto->pwrite += size;
    }
    else
    {
        // Should never happen because then we have frames which doesn't match
        memcpy(proto->rx_buffer + proto->pwrite, data, remaining);
        proto->pwrite += remaining;
    }
}

//------------------------------------------------------------------------------
void protocolSend(Protocol *proto, UsbFrame *frame)
{
    uint8_t *p = (uint8_t *)frame;
    uint8_t crc = 0;

    for(uint32_t i=0; i<(frame->length+4); i++)
    {
        crc ^= p[i];
    }
    frame->crc = crc;
    if(proto->send != 0)
    {
        proto->send((uint8_t *)frame, frame->length+4);
    }
}

//------------------------------------------------------------------------------
UsbFrame *protocolAllocFrame(Protocol *proto)
{
    return (UsbFrame *)proto->tx_buffer;
}

//------------------------------------------------------------------------------
void protocolService(Protocol *proto)
{
    if(proto->pwrite > 3)
    {
        UsbFrame *frame = (UsbFrame *)proto->rx_buffer;
        if(proto->pwrite >= (frame->length + 4))
        {
            if(proto->onFrameReceived)
            {
                proto->onFrameReceived(proto, frame);
            }
            proto->pwrite = 0;
        }
    }
}
