/**
 * @file   frame.h
 * @date   30.04.2020
 * @author andreas
 * @brief  
 */

#ifndef USB_FRAME_H_
#define USB_FRAME_H_

#include <stdint.h>

typedef struct UsbFrame_ UsbFrame;
typedef enum UsbCommands_ UsbCommands;
typedef enum UsbFlags_ UsbFlags;

enum UsbCommands_
{
    CMD_INVALID = 0,
    CMD_GET_VERSION,
    CMD_READ_COUNTER,
    CMD_RESET_COUNTER,
    CMD_RESET,
    CMD_END
};

enum UsbFlags_
{
    FRAME_FLAG_ACK = 0x1,
    FRAME_FLAG_NAK = 0x2
};

struct UsbFrame_
{
    uint8_t command;
    uint8_t flags;
    uint8_t length;
    uint8_t crc;
    uint8_t data[];
};

#define usbFrameSetCommand(frame, cmd)         (frame)->command = (cmd)
#define usbFrameGetCommand(frame)              ((frame)->command)
#define usbFrameSetFlags(frame, flags)         (frame)->flags = (flags)
#define usbFrameGetFlags(frame)                ((frame)->flags)
#define usbFrameSetLength(frame, len)          ((frame)->length = (len))

void usbFrameInit(UsbFrame *frame, UsbCommands cmd, UsbFlags flags);
void usbFrameAddData(UsbFrame *frame, const uint8_t *data, uint32_t size);

#endif /* FRAME_H_ */
