/**
 * @file   communication.c
 * @date   30.04.2020
 * @author andreas
 * @brief  
 */

#include "comm.h"
#include "usb_frame.h"
#include "debug.h"

#include "board.h"
#include "version.h"

#include "tusb.h"

#include <string.h>

static IP *ip_;

//------------------------------------------------------------------------------
static void commProcGetVersion(Protocol *proto, UsbFrame *frame)
{
    UsbFrame *resp = protocolAllocFrame(proto);

    usbFrameInit(resp, frame->command, FRAME_FLAG_ACK);
    usbFrameAddData(resp, (const uint8_t *)BUILD_VERSION_STR, strlen(BUILD_VERSION_STR));

    protocolSend(proto, resp);
}

//------------------------------------------------------------------------------
static void commProcSend(Protocol *proto, UsbFrame *frame)
{
    UsbFrameData *usbdata = (UsbFrameData *)frame->data;
    uint8_t len = frame->length - 2;

    ipSend(ip_, 0xffffffff, frame->data, frame->length);
/*
    Frame *tx = datalinkGetFreeTxFrame(datalink);
    frameSetDestination(tx, usbdata->remote_address);
    frameSetFlags(tx, 0);
    frameSetData(tx, usbdata->data, len);
    datalinkAddTxFrame(datalink, tx);
*/
    UsbFrame *resp = protocolAllocFrame(proto);
    usbFrameInit(resp, frame->command, FRAME_FLAG_ACK);
    protocolSend(proto, resp);
}

//------------------------------------------------------------------------------
static void commProcSetAddress(Protocol *proto, UsbFrame *frame)
{
    /*
    uint16_t *address = (uint16_t *)frame->data;
    datalinkSetAddress(datalink, *address);
     */
    UsbFrame *resp = protocolAllocFrame(proto);
    usbFrameInit(resp, frame->command, FRAME_FLAG_ACK);
    protocolSend(proto, resp);
}

//------------------------------------------------------------------------------
static void commProcReset(Protocol *proto, UsbFrame *frame)
{
    UsbFrame *resp = protocolAllocFrame(proto);
    usbFrameInit(resp, frame->command, FRAME_FLAG_ACK);
    protocolSend(proto, resp);

    NVIC_SystemReset();
}

//------------------------------------------------------------------------------
void commInit(IP *ip)
{
    ip_ = ip;
}

//------------------------------------------------------------------------------
void commSend(uint8_t *data, uint32_t length)
{
    tud_cdc_write(data, length);
    tud_cdc_write_flush();
}

//------------------------------------------------------------------------------
void commOnUsbFrameReceived(Protocol *proto, UsbFrame *frame)
{
    debug("Command: %d", frame->command);
    switch (frame->command)
    {
        case CMD_GET_VERSION:
            commProcGetVersion(proto, frame);
            break;

        case CMD_SEND:
            commProcSend(proto, frame);
            break;

        case CMD_SET_ADDRESS:
            commProcSetAddress(proto, frame);
            break;

        case CMD_RESET:
            NVIC_SystemReset();
            commProcReset(proto, frame);
            break;

        default:
        {
            debug("Unknown Command: %d", frame->command);

            UsbFrame *resp = protocolAllocFrame(proto);
            usbFrameInit(resp, frame->command, FRAME_FLAG_NAK);
            protocolSend(proto, resp);
        }
    }
}

