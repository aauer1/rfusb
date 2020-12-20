/**
 * @file   webusb.c
 * @date   23.04.2020
 * @author andreas
 * @brief  
 */

#include "usb/webusb.h"
#include "usb/usb_descriptors.h"

#include "led.h"
#include "debug.h"

#define URL  "tse902.emicrotec.at"

struct WebUSB_
{
    Protocol *proto;
    uint8_t connected_;

    uint8_t tx_buffer[1024];
    uint32_t tx_size;
    uint32_t tx_count;
};

const tusb_desc_webusb_url_t desc_url =
{
  .bLength         = 3 + sizeof(URL) - 1,
  .bDescriptorType = 3, // WEBUSB URL type
  .bScheme         = 1, // 0: http, 1: https
  .url             = URL
};

static struct WebUSB_ webusb;

//------------------------------------------------------------------------------
void webusbInit(Protocol *proto)
{
    webusb.connected_ = 0;
    webusb.proto = proto;
    webusb.tx_size = 0;
    webusb.tx_count = 0;
}

//------------------------------------------------------------------------------
bool webusbControlRequest(uint8_t rhport, tusb_control_request_t const *request)
{
    debug("Request: %d", request->bRequest);
    switch (request->bRequest)
    {
        case VENDOR_REQUEST_WEBUSB:
            // match vendor request in BOS descriptor
            // Get landing page url
            return tud_control_xfer(rhport, request, (void*) &desc_url, desc_url.bLength);

        case VENDOR_REQUEST_MICROSOFT:
            if(request->wIndex == 7)
            {
                // Get Microsoft OS 2.0 compatible descriptor
                uint16_t total_len;
                memcpy(&total_len, desc_ms_os_20+8, 2);

                return tud_control_xfer(rhport, request, (void*) desc_ms_os_20, total_len);
            }
            else
            {
                return false;
            }

        case 0x22:
            // Webserial simulate the CDC_REQUEST_SET_CONTROL_LINE_STATE (0x22) to
            // connect and disconnect.
            webusb.connected_ = (request->wValue != 0);
            debug("USB Connected: %d", webusb.connected_);

            // response with status OK
            return tud_control_status(rhport, request);

        default:
            // stall unknown request
            return false;
    }

    return true;
}

//------------------------------------------------------------------------------
uint32_t webusbSend(uint8_t *data, uint32_t size)
{
    if(size > sizeof(webusb.tx_buffer))
    {
        return 0;
    }

    if(webusb.connected_ == 0)
    {
        return 0;
    }

    memcpy(webusb.tx_buffer, data, size);
    webusb.tx_size = size;
    webusb.tx_count = 0;

    return size;
}

//------------------------------------------------------------------------------
void webusbService(void)
{
    if(webusb.connected_)
    {
        if(tud_vendor_available())
        {
            uint8_t buf[64];
            uint32_t count = tud_vendor_read(buf, sizeof(buf));

            protocolAddData(webusb.proto, buf, count);
        }

        if(webusb.tx_size > webusb.tx_count)
        {
            uint32_t diff = webusb.tx_size - webusb.tx_count;
            uint32_t available = tud_vendor_write_available();
            if(available > diff)
            {
                tud_vendor_write(webusb.tx_buffer + webusb.tx_count, diff);
                webusb.tx_count += diff;
            }
            else
            {
                tud_vendor_write(webusb.tx_buffer + webusb.tx_count, available);
                webusb.tx_count += available;
            }
        }
    }
}
