/**
 * @file   comm.h
 * @date   05.08.2020
 * @author andreas
 * @brief  
 */

#ifndef COMM_H_
#define COMM_H_

#include "usb_frame.h"
#include "protocol.h"
#include "datalink.h"

void commInit(Datalink *link);
void commSend(uint8_t *data, uint32_t length);
void commOnUsbFrameReceived(Protocol *proto, UsbFrame *frame);

#endif /* COMM_H_ */
