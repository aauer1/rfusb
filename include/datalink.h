#ifndef DATALINK_H
#define DATALINK_H

#include "timer.h"
#include "rfm22b.h"

#define BROADCAST_ADDR              0xFFFF

#define MAX_DATA_LENGTH             32
#define MAX_TX_FRAME_NUM            8
#define MAX_RX_FRAME_NUM            8

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
    FLAG_ACK = 0,
};

struct Frame_
{
    unsigned short dest_addr;
    unsigned short src_addr;
    unsigned char  length;
    unsigned char  sequence;
    unsigned char  flags;
    unsigned char  rssi;
    unsigned char  data[MAX_DATA_LENGTH];
};

struct Datalink_
{
// public:
    unsigned short src_addr;
    
    unsigned short tx_errors;
    unsigned char  retransmission;
    void (*onReceive)(Datalink *proto);
    void (*onFailure)(Datalink *proto);
    
// private:
    Frame tx_frames[MAX_TX_FRAME_NUM];
    unsigned short tx_sent;
    unsigned char tx_frame_first;
    unsigned char tx_frame_last;

    Frame rx_frames[MAX_RX_FRAME_NUM];
    unsigned char rx_frame_first;
    unsigned char rx_frame_last;

    Timer tx_timeout;
    unsigned char tx_count;
    unsigned char tx_retransmit;
    
    unsigned char rssi;
    unsigned char temp_rssi;
    unsigned char sequence;
    unsigned char state;
};

#define frameSetDestination(frame, value)           (frame)->dest_addr = (value)
#define frameSetSource(frame, value)                (frame)->src_addr = (value)
#define frameSetLength(frame, value)                (frame)->length = (value)
#define frameSetData(frame, d, len)                 memcpy((frame)->data, (d), (len)); (frame)->length = len
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
#define datalinkSetAddress(proto, value)            (proto)->src_addr = (value); radioSetRxAddress((proto)->src_addr, 0xFFFF)

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

