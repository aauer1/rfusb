/**
 * @file   webusb.h
 * @date   23.04.2020
 * @author andreas
 * @brief  
 */

#ifndef WEBUSB_H_
#define WEBUSB_H_

#include "tusb.h"
#include "protocol.h"

void webusbInit(Protocol *proto);
bool webusbControlRequest(uint8_t rhport, tusb_control_request_t const *request);
uint32_t webusbSend(uint8_t *data, uint32_t size);
void webusbService(void);

#endif /* WEBUSB_H_ */
