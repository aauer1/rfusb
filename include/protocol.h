/**
 * @file   protocol.h
 * @date   23.04.2020
 * @author andreas
 * @brief  
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include "frame.h"

#include <stdint.h>

#define PROTOCOL_MAX_FRAME_SIZE     532

typedef struct Protocol_ Protocol;

struct Protocol_
{
    uint8_t rx_buffer[PROTOCOL_MAX_FRAME_SIZE];
    uint8_t tx_buffer[PROTOCOL_MAX_FRAME_SIZE];
    uint16_t pwrite;

    void (*onFrameReceived)(Protocol *proto, Frame *frame);
};

void protocolInit(Protocol *proto);
void protocolSetCallback(Protocol *proto, void (*cb)(Protocol *, Frame *));
void protocolAddData(Protocol *proto, uint8_t *data, uint16_t size);
void protocolSend(Protocol *proto, Frame *frame);
Frame *protocolAllocFrame(Protocol *proto);
void protocolService(Protocol *proto);

#endif /* PROTOCOL_H_ */
