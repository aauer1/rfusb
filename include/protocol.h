/**
 * @file   protocol.h
 * @date   23.04.2020
 * @author andreas
 * @brief  
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <stdint.h>
#include "usb_frame.h"

#define PROTOCOL_MAX_FRAME_SIZE     1536

typedef struct Protocol_ Protocol;

struct Protocol_
{
    uint8_t rx_buffer[PROTOCOL_MAX_FRAME_SIZE];
    uint8_t tx_buffer[PROTOCOL_MAX_FRAME_SIZE];
    uint16_t pwrite;

    void (*send)(uint8_t *data, uint32_t len);
    void (*onFrameReceived)(Protocol *proto, UsbFrame *frame);
};

void protocolInit(Protocol *proto);
void protocolSetCallback(Protocol *proto, void (*cb)(Protocol *, UsbFrame *));
void protocolSetSendCallback(Protocol *proto, void (*cb)(uint8_t *, uint32_t));
void protocolAddData(Protocol *proto, uint8_t *data, uint16_t size);
void protocolSend(Protocol *proto, UsbFrame *frame);
UsbFrame *protocolAllocFrame(Protocol *proto);
void protocolService(Protocol *proto);

#endif /* PROTOCOL_H_ */
