#include <stm32l0xx.h>

#include "datalink.h"
#include "rfm22b.h"
#include "led.h"
#include "debug.h"

#include <string.h>

//------------------------------------------------------------------------------
static void adjustRfPower(unsigned char rssi)
{
#ifdef ADAPTIVE_RF_POWER
    unsigned char power = radioGetPower();

    if(rssi == 0)
        return;
        
    if(power > 0 && rssi > (MIN_RSSI_LEVEL + RSSI_HYSTERESIS))
    {
        power -= 1;
        radioSetPower(power);
    }
    else if(power < 7 && rssi < (MIN_RSSI_LEVEL - RSSI_HYSTERESIS))
    {
        power += 1;
        radioSetPower(power);
    }
    
#ifdef DEBUG
    debug("RSSI level: %s", rssi);
    debug("Power level: %s", power);
#endif
#endif
}

//------------------------------------------------------------------------------
static void receive(Datalink *proto)
{
    Frame *frame = &proto->rx_frames[proto->rx_frame_first];
    uint8_t rssi;
/*
    ATOMIC_BLOCK()
    {
        rssi = radioRead(RFM22_RSSI);
    }
    if(proto->temp_rssi < rssi)
    {
        proto->temp_rssi = rssi;
    }
*/
    uint16_t len = radioReceive((uint8_t *)frame, MAX_DATA_LENGTH + 12);
    if(len == 0)
    {
        return;
    }

    proto->rssi = proto->temp_rssi;
    proto->temp_rssi = 0;

    // Process packet acknowledge
    if(proto->tx_sent)
    {
        if(proto->tx_frames[proto->tx_frame_last].sequence == frame->sequence &&
          (proto->rx_frames[proto->rx_frame_first].flags & (1 << FLAG_ACK)))
        {
            proto->tx_sent &= ~(1 << proto->tx_frame_last);
            proto->tx_frame_last++;
            if(proto->tx_frame_last == MAX_TX_FRAME_NUM)
            {
                proto->tx_frame_last = 0;
            }
        }
    }
/*
    adjustRfPower(frame->rssi);
    debug("RSSI level: %d", proto->rssi);
*/

    proto->rx_frame_first++;
    if(proto->rx_frame_first == MAX_RX_FRAME_NUM)
    {
        proto->rx_frame_first = 0;
    }

    if(proto->onReceive)
    {
        proto->tx_errors = 0;
        proto->onReceive(proto, proto->userarg);
    }
}

//------------------------------------------------------------------------------
static uint8_t send(Datalink *proto)
{
    Frame *frame; 
    
    if(proto->tx_frame_first != proto->tx_frame_last)
    {
        uint16_t status = radioStatus();
        if((status & RFM22_RSSI_INDICATOR) == 0)
        {
            radioReadyMode();

            frame = &proto->tx_frames[proto->tx_frame_last];
            if(!(frame->flags & (1 << FLAG_ACK)))
            {
                frame->sequence = proto->sequence++;
            }

            frameSetRSSI(frame, proto->rssi);
            radioSetHeader(frame->dest_addr);
            uint8_t *data = (uint8_t *)frame;
            data += 5;
            radioTransmit(data, frame->length+7);

            if(frame->dest_addr == BROADCAST_ADDR ||
               (frame->flags & ((1 << FLAG_ACK) | (1 << FLAG_DGRAM))))
            {
                proto->tx_frame_last++;
                if(proto->tx_frame_last == MAX_TX_FRAME_NUM)
                {
                    proto->tx_frame_last = 0;
                }
            }
            else
            {
                timerSet(&(proto->tx_timeout), 500);
                proto->tx_sent |= (1 << proto->tx_frame_last);
            }

            return 1;
        }
        else
        {
            debug("Channel NOT CLEAR");
        }
    }

    return 0;
}

//------------------------------------------------------------------------------
static void datalinkSetModeRf(Datalink *proto, DatalinkState value)
{
    proto->state = value;
    switch(value)
    {
        case STATE_IDLE:
            debug("Idle state");
            radioReadyMode();
            break;

        case STATE_RX:
            debug("RX state");
            radioRxMode();
            break;

        case STATE_TX:
            break;
            
        default:
            break;
    }
}

//------------------------------------------------------------------------------
void datalinkInit(Datalink *proto)
{
    proto->tx_frame_first = 0;
    proto->tx_frame_last  = 0;
    proto->rx_frame_first = 0;
    proto->rx_frame_last  = 0;

    proto->sequence  = 0;
    proto->tx_sent   = 0;
    proto->tx_errors = 0;
    proto->rssi      = 0;
    proto->temp_rssi = 0;
    
    proto->tx_retransmit = 0;
    proto->tx_count = 0;

    radioSetRxAddress(proto->src_addr, 0xFFFF);
}

//------------------------------------------------------------------------------
void datalinkReset(Datalink *proto)
{
    proto->tx_frame_first = 0;
    proto->tx_frame_last  = 0;
    proto->rx_frame_first = 0;
    proto->rx_frame_last  = 0;

    proto->sequence  = 0;
    proto->tx_sent   = 0;
    proto->tx_errors = 0;
    proto->rssi      = 0;
    proto->temp_rssi = 0;

    proto->tx_retransmit = 0;
    proto->tx_count = 0;
}

//------------------------------------------------------------------------------
void datalinkSetMode(Datalink *proto, DatalinkState value)
{
    datalinkSetModeRf(proto, value);
}

//------------------------------------------------------------------------------
void datalinkService(Datalink *proto)
{
    if(proto->tx_frame_first != proto->tx_frame_last)
    {
        debug("Frame(s) to send");
        if(!(proto->tx_sent & (1 << proto->tx_frame_last)))
        {
            debug("TX_Frame (1)");

            send(proto);
            proto->state = STATE_TX;
            proto->tx_count = 0;
            proto->tx_retransmit = 0;
        }
        else if(timerExpired(&(proto->tx_timeout)))
        {
            if(proto->tx_count < proto->retransmission)
            {
                radioSetPower(0x07);
                proto->tx_retransmit = 1;
                proto->tx_count++;
            }
            else
            {
                debug("Retransmission error");

                proto->tx_errors++;
                if(proto->onFailure)
                {
                    proto->onFailure(proto, proto->userarg);
                }
                proto->tx_sent &= ~(1 << proto->tx_frame_last);
                proto->tx_frame_last++;
                if(proto->tx_frame_last == MAX_TX_FRAME_NUM)
                {
                    proto->tx_frame_last = 0;
                }

                debug("Full RF power");
                datalinkSetMode(proto, STATE_RX);
            }
        }
    }

    if(proto->tx_retransmit)
    {
        debug("Retransmission %d", proto->tx_count+1);

        uint8_t ret = 0;
        ret = send(proto);
        proto->state = STATE_TX;
        if(ret == 1)
        {
            proto->tx_retransmit = 0;
        }
    }

    if(proto->state != STATE_RX)
    {
        debug("switch to RX mode");
        datalinkSetMode(proto, STATE_RX);
    }

    if(proto->state == STATE_RX)
    {
        receive(proto);
    }
}

//------------------------------------------------------------------------------
Frame *datalinkGetFreeTxFrame(Datalink *proto)
{
    proto->tx_sent &= ~(1 << proto->tx_frame_first);
    proto->tx_frames[proto->tx_frame_first].src_addr = proto->src_addr;
    return &proto->tx_frames[proto->tx_frame_first];
}

//------------------------------------------------------------------------------
void datalinkAddTxFrame(Datalink *proto, Frame *packet)
{
    proto->tx_frame_first++;
    if(proto->tx_frame_first == MAX_TX_FRAME_NUM)
    {
        proto->tx_frame_first = 0;
    }
}
