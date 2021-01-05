/**
 * @file   ip.c
 * @date   03.01.2021
 * @author andreas
 * @brief  
 */

#include "ip.h"

#include "debug.h"

//-----------------------------------------------------------------------------
static void onReceive(Datalink *datalink, void *userarg)
{
    IP *ip = (IP *)userarg;
    Frame *frame = datalinkGetRxFrame(datalink);
    uint32_t len = frameGetLength(frame);
    debug("Len: %d", len);
    debug("Flags: %x", frameGetFlags(frame));

    if(frameGetFlags(frame) & (1 << FLAG_FIRST))
    {
        memcpy(ip->rx_buffer, (uint8_t *)frame, len+5);
        ip->rx_count = len+5;
    }
    else
    {
        memcpy(ip->rx_buffer + ip->rx_count, frame->data, len-7);
        ip->rx_count += (len-7);
    }

    if(frameGetFlags(frame) & (1 << FLAG_LAST))
    {
        if(ip->onIpReceived)
        {
            ip->onIpReceived(ip, ip->rx_buffer+12, ip->rx_count-12);
        }
        ip->rx_count = 0;
    }

    datalinkReleaseRxFrame(datalink);
}

//-----------------------------------------------------------------------------
static void onFailure(Datalink *datalink, void *userarg)
{
    (void)datalink;
    (void)userarg;
}

//-----------------------------------------------------------------------------
void ipInit(IP *ip, Datalink *datalink)
{
    ip->datalink = datalink;
    ip->datalink->userarg = ip;
    ip->datalink->onReceive = onReceive;
    ip->datalink->onFailure = onFailure;
    ip->rx_count = 0;

    datalinkInit(datalink);
}

//-----------------------------------------------------------------------------
void ipSetCallback(IP *ip, void (*cb)(IP *, uint8_t *, uint32_t))
{
    ip->onIpReceived = cb;
}

//-----------------------------------------------------------------------------
void ipSend(IP *ip, uint32_t dest, uint8_t *data, uint32_t size)
{
    uint32_t diff = 0;
    debug("Send IP packet: %d", size);
    for(uint32_t i=0; i<size; i+=MAX_DATA_LENGTH)
    {
        Frame *frame = datalinkGetFreeTxFrame(ip->datalink);
        frameSetDestination(frame, dest);
        frameSetSource(frame, ip->datalink->src_addr);
        frameSetFlags(frame, 1 << FLAG_DGRAM);
        if(i == 0)
        {
            frameSetFlags(frame, frameGetFlags(frame) | (1 << FLAG_FIRST));
        }

        diff = size - i;
        if(diff > MAX_DATA_LENGTH)
        {
            frameSetData(frame, data+i, MAX_DATA_LENGTH);
        }
        else
        {
            frameSetFlags(frame, frameGetFlags(frame) | (1 << FLAG_LAST));
            frameSetData(frame, data+i, diff);
        }

        datalinkAddTxFrame(ip->datalink, frame);
    }
}
