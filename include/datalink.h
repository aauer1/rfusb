#ifndef DATALINK_H
#define DATALINK_H

#include "timer.h"
#include "rfm22b.h"

#include <string.h>

#define BROADCAST_ADDR              0xFFFFFFFF

#define MAX_DATA_LENGTH             128
#define MAX_TX_FRAME_NUM            16
#define MAX_RX_FRAME_NUM            8

#define MAX_PACKET_SIZE             1536

#ifdef ADAPTIVE_RF_POWER
#   define MIN_RSSI_LEVEL              100
#   define RSSI_HYSTERESIS             20
#endif

typedef struct Datalink_ Datalink;
typedef struct Frame_ Frame;
typedef enum DatalinkState_ DatalinkState;
typedef enum DatalinkFlags_ DatalinkFlags;

enum DatalinkState_
{
    STATE_IDLE = 0,
    STATE_RX,
    STATE_TX,
};

enum DatalinkFlags_
{
    FLAG_ACK   = 0,
    FLAG_NAK   = 1,
    FLAG_DGRAM = 2,

    FLAG_FIRST = 4,
    FLAG_LAST  = 5,
};

struct Frame_
{
    uint32_t dest_addr;
    uint8_t  length;
    uint8_t  sequence;
    uint8_t  flags;
    uint8_t  rssi;
    uint32_t src_addr;
    uint8_t  data[MAX_DATA_LENGTH];
};

struct Datalink_
{
// public:
    uint32_t src_addr;
    
    uint8_t tx_errors;
    uint8_t retransmission;
    void (*onReceive)(Datalink *proto, void *userarg);
    void (*onFailure)(Datalink *proto, void *userarg);
    void *userarg;
    
// private:
    Frame tx_frames[MAX_TX_FRAME_NUM];
    uint8_t tx_sent;
    uint8_t tx_frame_first;
    uint8_t tx_frame_last;

    Frame rx_frames[MAX_RX_FRAME_NUM];
    uint8_t rx_frame_first;
    uint8_t rx_frame_last;

    uint8_t rx_buffer[MAX_PACKET_SIZE];
    uint16_t rx_buffer_size;

    Timer tx_timeout;
    uint8_t tx_count;
    uint8_t tx_retransmit;
    
    uint8_t rssi;
    uint8_t temp_rssi;
    uint8_t sequence;
    uint8_t state;
};

#define frameSetDestination(frame, value)           (frame)->dest_addr = (value)
#define frameSetSource(frame, value)                (frame)->src_addr = (value)
#define frameSetLength(frame, value)                (frame)->length = (value)
#define frameSetData(frame, d, len)                 memcpy((frame)->data, (d), (len)); frame->length = (len)
#define frameSetSequence(frame, value)              (frame)->sequence = (value)
#define frameSetFlags(frame, value)                 (frame)->flags = (value)
#define frameSetRSSI(frame, value)                  (frame)->rssi = (value)

#define frameGetDestination(frame)                  ((frame)->dest_addr)
#define frameGetSource(frame)                       ((frame)->src_addr)
#define frameGetLength(frame)                       ((frame)->length)
#define frameGetData(frame)                         ((frame)->data)
#define frameGetSequence(frame)                     ((frame)->sequence)
#define frameGetFlags(frame)                        ((frame)->flags)
#define frameGetRSSI(frame)                         ((frame)->rssi)

#define datalinkSetRetransmission(proto, value)     (proto)->retransmission = (value)
//#define datalinkSetAddress(proto, value)            (proto)->src_addr = (value); radioSetRxAddress((proto)->src_addr, 0xFFFF)

#define datalinkTxFrames(proto)                     ((proto)->tx_frame_first != (proto)->tx_frame_last)
#define datalinkRxFrames(proto)                     ((proto)->rx_frame_first != (proto)->rx_frame_last)

#define datalinkGetMode(proto)                      ((proto)->state)
#define datalinkGetRSSI(proto)                      ((proto)->rssi)
#define datalinkGetInterface(proto)                 ((proto)->interface)

#define datalinkGetRxFrame(proto)                   (datalinkRxFrames(proto) ? \
                                                        &(proto)->rx_frames[(proto)->rx_frame_last] : \
                                                        (Frame *)0)
#define datalinkReleaseRxFrame(proto)               (proto)->rx_frames[(proto)->rx_frame_last].rssi = 0; \
                                                        (proto)->rx_frame_last++; \
                                                        (proto)->rx_frame_last = \
                                                            ((proto)->rx_frame_last == MAX_RX_FRAME_NUM) ? \
                                                                0 : (proto)->rx_frame_last

void datalinkInit(Datalink *proto);
void datalinkReset(Datalink *proto);
void datalinkSetMode(Datalink *proto, DatalinkState value);
void datalinkService(Datalink *proto);
Frame *datalinkGetFreeTxFrame(Datalink *proto);
void datalinkAddTxFrame(Datalink *proto, Frame *frame);

#endif

