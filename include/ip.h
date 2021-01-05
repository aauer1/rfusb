/**
 * @file   ip.h
 * @date   03.01.2021
 * @author andreas
 * @brief  
 */

#ifndef IP_H_
#define IP_H_

#include "datalink.h"

typedef struct IP_ IP;

struct IP_
{
    Datalink *datalink;
    uint8_t  rx_buffer[1536];
    uint32_t rx_count;

    void (*onIpReceived)(IP *ip, uint8_t *data, uint32_t size);
};

void ipInit(IP *ip, Datalink *datalink);
void ipSetCallback(IP *ip, void (*cb)(IP *, uint8_t *, uint32_t));
void ipSend(IP *ip, uint32_t dest, uint8_t *data, uint32_t size);

#endif /* IP_H_ */
