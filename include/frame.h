/**
 * @file   frame.h
 * @date   30.04.2020
 * @author andreas
 * @brief  
 */

#ifndef FRAME_H_
#define FRAME_H_

#include <stdint.h>

typedef struct Frame_ Frame;
typedef enum Commands_ Commands;
typedef enum Flags_ Flags;

enum Commands_
{
    CMD_INVALID = 0,
    CMD_GET_VERSION,
    CMD_RECEIVE_CONFIG,
    CMD_SEND_CONFIG,
    CMD_RECEIVE_LOG,
    CMD_CLEAR_LOG,
    CMD_ERASE_FLASH,
    CMD_RESET,
    CMD_ERASE_APPLICATION,
    CMD_FIRMWARE_UPDATE,
    CMD_END
};

enum Flags_
{
    FLAG_ACK = 0x1,
    FLAG_NAK = 0x2
};

struct Frame_
{
    uint8_t command;
    uint8_t flags;
    uint16_t length;
    uint8_t crc;
    uint8_t data[];
};

#define frameSetCommand(frame, cmd)         (frame)->command = (cmd)
#define frameGetCommand(frame)              ((frame)->command)
#define frameSetFlags(frame, flags)         (frame)->flags = (flags)
#define frameGetFlags(frame)                ((frame)->flags)
#define frameSetLength(frame, len)       ((frame)->length = (len))

void frameInit(Frame *frame, Commands cmd, Flags flags);
void frameAddData(Frame *frame, const uint8_t *data, uint32_t size);

#endif /* FRAME_H_ */
