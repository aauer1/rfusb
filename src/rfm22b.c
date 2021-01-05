#include "rfm22b.h"

#include "gpio.h"
#include "utils.h"
#include "board.h"
#include "debug.h"
#include "led.h"

#include <stm32l0xx.h>
#include <stm32l0xx_hal.h>

#include <stddef.h>
#include <string.h>

#define radioReset()            gpioSet(config_->shdn)
#define radioReady()            gpioClear(config_->shdn)

#define radioSelect()           gpioClear(config_->cs)
#define radioDeselect()         gpioSet(config_->cs)

#define FIFO_ALMOST_EMPTY       16
#define FIFO_PACKET_SIZE        48
#define FIFO_REFILL_SIZE        32

static const RFM22B_RegEntry RFM22B_REGS[] =
{
    {RFM22_INTERRUPT_ENABLE2, 0x00},
    {RFM22_FUNCTION_CONTROL1, 0x01},
    {0x09, 0x7F},       // Cap = 12.5pF
    {0x0A, 0x05},		// Clk output is 2MHz
    {0x0B, 0x92},		// GPIO0 is for TX state
    {0x0C, 0x95},		// GPIO1 is for RX state output
    {0x0D, 0xF4},		// GPIO2 for RX data output
    {0x0E, 0x00},		// GPIO port use default value
	
    {0x0F, 0x70},		// NO ADC used
    {0x10, 0x00},		// no ADC used
    {0x12, 0x00},		// No temp sensor used
    {0x13, 0x00},		// no temp sensor used

//------------------------------------------------------------------------------
//  GFSK/FSK Receiver Settings
    {0x1C, 0x06},		// IF filter bandwidth
    {0x1D, 0x40},		// AFC Loop
    {0x1E, 0x0A},       // AFC timing
    {0x1F, 0x03},

    {0x20, 0x68},		// clock recovery
    {0x21, 0x01},		// clock recovery
    {0x22, 0x3A},		// clock recovery
    {0x23, 0x93},		// clock recovery
    {0x24, 0x02},		// clock recovery timing
    {0x25, 0x5F},		// clock recovery timing

    {0x2A, 0x28},
    {0x69, 0x60},

//------------------------------------------------------------------------------
//  OOK Receiver settings
    {0x2C, 0x00},
    {0x2D, 0x00},
    {0x2E, 0x00},

//------------------------------------------------------------------------------
//  Packet handling
    {0x30, 0xA8},		// Data access control

    {0x32, 0xFF},		// Header control

    {0x33, 0x42},		// Header 3, 2, 1, 0 used for head length, fixed packet length, synchronize word length 3, 2,

    {0x34, 0x08},	        // 32 nibble = 16 byte preamble
    {0x35, 0x22},		// 0x35 need to detect 20bit preamble
    {0x36, 0x2D},		// synchronize word
    {0x37, 0xD4},
    {0x38, 0x00},
    {0x39, 0x00},
    {0x3A, 0x00},		// set tx header 3
    {0x3B, 0x00},		// set tx header 2
    {0x3C, 0x00},		// set tx header 1
    {0x3D, 0x00},		// set tx header 0
    {0x3E, 0x00},       // set packet length to 17 bytes

    {0x3F, 0x00},		// set rx header
    {0x40, 0x00},
    {0x41, 0x00},
    {0x42, 0x00},
    {0x43, 0xFF},		// check all bits
    {0x44, 0xFF},		// Check all bits
    {0x45, 0xFF},		// check all bits
    {0x46, 0xFF},		// Check all bits

    {0x56, 0x01},

    {0x6D, 0x0F},		// Tx power to max

//------------------------------------------------------------------------------
//  TX Datarate settings
    {0x6E, 0x09},		// TX data rate 1
    {0x6F, 0xD5},		// TX data rate 0
    {0x70, 0x0C},		// GFSK
    {0x58, 0x80},       // undocumented

//------------------------------------------------------------------------------	
//  RX/TX Frequency settings
    {0x71, 0x23},		// GFSK
    {0x72, 0x40},		// Frequency deviation setting to 40K

    {0x73, 0x00},		// No frequency offset
    {0x74, 0x00},		// No frequency offset

    {0x75, 0x73},       // frequency set to 868.4MHz
    {0x76, 0x69},       // frequency set to 868.4MHz
    {0x77, 0x00},       // frequency set to 868.4Mhz

    {0x79, 0x00},		// no frequency hopping
    {0x7A, 0x00},		// no frequency hopping

    {0x7D, 15},         // TX FIFO almost empty threshold
    {0x7E, 47},         // RX FIFO almost full threshold

//------------------------------------------------------------------------------	
//	{0x5A, 0x7F},
//	{0x59, 0x40},

//	{0x6A, 0x0B},
//  {0x68, 0x04},       // undocumented

    {0x59, 0x00},       // SI4432 Errata 9
    {0x5A, 0x03},       // SI4432 Errata 9
    {0x66, 0x02},       // SI4432 Errata 9

    {0x65, 0xA1},       // SI4432 Errata 10 (PLL Tuning)

    {-1, 0x00},
};

#define STATE_TX            1
#define STATE_RX            2

static RFM22BConfig *config_;

static uint8_t rx_buffer_[256];
static uint8_t rx_count_ = 0;
static uint8_t rx_valid_ = 0;

static uint8_t *tx_buffer_ = 0;
static uint8_t tx_size_   = 0;
static uint8_t tx_offset_ = 0;
static uint8_t tx_done_   = 1;

static uint8_t state_ = 0;

//------------------------------------------------------------------------------
static uint8_t spiReadWrite(unsigned char byte)
{
    uint8_t tx = byte;
    uint8_t rx;

    HAL_SPI_TransmitReceive(&config_->hspi, &tx, &rx, 1, 100);

    return rx;
}

//------------------------------------------------------------------------------
void radioInit(RFM22BConfig *config)
{
    config_ = config;
    
    __HAL_RCC_SPI1_CLK_ENABLE();

    config->hspi.Instance = config->spi;
    config->hspi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    config->hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
    config->hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
    config->hspi.Init.DataSize = SPI_DATASIZE_8BIT;
    config->hspi.Init.Direction = SPI_DIRECTION_2LINES;
    config->hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    config->hspi.Init.Mode = SPI_MODE_MASTER;
    config->hspi.Init.NSS  = SPI_NSS_SOFT;
    config->hspi.Init.TIMode = SPI_TIMODE_DISABLE;
    config->hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    config->hspi.Init.CRCPolynomial = 1;
    HAL_SPI_Init(&config->hspi);

    radioDeselect();
    
    radioReset();
    HAL_Delay(10);
    radioReady();
    HAL_Delay(50);
}

//------------------------------------------------------------------------------
void radioIrqHandler(uint16_t pin)
{
    uint16_t status = radioStatus();

    if(state_ == STATE_RX)
    {
        if(rx_valid_ == 0)
        {
            if(status & RFM22_VALID_PACKET)
            {
                uint8_t len = radioRead(RFM22_RECEIVED_PACKET_LENGTH) + 5;
                uint8_t diff = len - rx_count_;
                radioReadBurst(RFM22_FIFO_ACCESS, rx_buffer_ + rx_count_, diff);
                rx_count_ += diff;
                rx_valid_ = 1;
                ledToggle(LED1);
            }
            else if(status & RFM22_RX_ALMOST_FULL)
            {
                radioReadBurst(RFM22_FIFO_ACCESS, rx_buffer_ + rx_count_, FIFO_PACKET_SIZE);
                rx_count_ += FIFO_PACKET_SIZE;
            }
        }
    }
    else if(state_ == STATE_TX)
    {
        if(tx_done_ == 0)
        {
            if(status & RFM22_TX_ALMOST_EMPTY)
            {
                if(tx_size_ > FIFO_REFILL_SIZE)
                {
                    radioWriteBurst(RFM22_FIFO_ACCESS, tx_buffer_ + tx_offset_, FIFO_REFILL_SIZE);
                    tx_offset_ += FIFO_REFILL_SIZE;
                    tx_size_ -= FIFO_REFILL_SIZE;
                }
                else if(tx_size_ > 0)
                {
                    radioWriteBurst(RFM22_FIFO_ACCESS, tx_buffer_ + tx_offset_, tx_size_);
                    tx_offset_ += tx_size_;
                    tx_size_ = 0;
                }
            }
            else if(status & RFM22_PACKET_SENT)
            {
                tx_done_ = 1;
            }
        }
    }
}

//------------------------------------------------------------------------------
void radioInitRegs(const RFM22B_RegEntry *regs)
{
    unsigned char i=0; 
    if(regs == 0)
    {
        regs = RFM22B_REGS;
    }
    for(i = 0; regs[i].addr != -1; i++)
    {
        radioSelect();
        spiReadWrite(regs[i].addr | 0x80);
        spiReadWrite(regs[i].value);
        radioDeselect();
    }
}

//------------------------------------------------------------------------------
unsigned char radioInterrupt(void)
{
    return (gpioRead(config_->irq) == 0);
}

//------------------------------------------------------------------------------
void radioShutdown(void)
{
    radioReset();
}

//------------------------------------------------------------------------------
void radioWakeup(void)
{
    radioReady();
}

//------------------------------------------------------------------------------
unsigned char radioRead(unsigned char addr)
{
    unsigned char ret;

    radioSelect();
    HAL_Delay(1);
    spiReadWrite(addr & 0x7F);
    ret = spiReadWrite(0xaa);
    //HAL_Delay(1);
    radioDeselect();
    
    return ret;
}

//------------------------------------------------------------------------------
void radioWrite(unsigned char addr, unsigned char value)
{
    radioSelect();
    spiReadWrite(addr | 0x80);
    spiReadWrite(value);
    radioDeselect();
}

//------------------------------------------------------------------------------
void radioReadBurst(unsigned char addr, unsigned char *data, unsigned char len)
{
    unsigned char i=0; 
    
    radioSelect();
    spiReadWrite(addr & 0x7F);
    for(i=0; i<len; i++)
    {
        data[i] = spiReadWrite(0x00);
    }
    radioDeselect();
}

//------------------------------------------------------------------------------
void radioWriteBurst(unsigned char addr, unsigned char *data, unsigned char len)
{
    unsigned char i=0; 
    
    radioSelect();
    spiReadWrite(addr | 0x80);
    for(i=0; i<len; i++)
    {
        spiReadWrite(data[i]);
    }
    radioDeselect();
}

//------------------------------------------------------------------------------
uint8_t radioTransmit(uint8_t *data, uint16_t size)
{
    uint16_t status = 0;
    UNUSED(status);

    radioWrite(RFM22_FUNCTION_CONTROL2, 0x01);  // clear FIFOs
    radioWrite(RFM22_FUNCTION_CONTROL2, 0x00);  // clear FIFOs (reset)

    radioWrite(RFM22_TRANSMIT_PACKET_LENGTH, size);
    debug("Packet length: %d", size);

    status = radioStatus();
    radioSetInterrupts(RFM22_PACKET_SENT | RFM22_TX_ALMOST_EMPTY);

    tx_done_ = 0;
    tx_buffer_ = data;
    tx_size_ = size;
    state_ = STATE_TX;
    if(tx_size_ > FIFO_PACKET_SIZE)
    {
        radioWriteBurst(RFM22_FIFO_ACCESS, tx_buffer_, FIFO_PACKET_SIZE);
        tx_size_  -= FIFO_PACKET_SIZE;
        tx_offset_ = FIFO_PACKET_SIZE;
    }
    else
    {
        radioWriteBurst(RFM22_FIFO_ACCESS, tx_buffer_, tx_size_);
        tx_offset_ = tx_size_;
        tx_size_ = 0;
    }
    radioWrite(RFM22_FUNCTION_CONTROL1, 0x09);

    while(tx_done_ == 0)
    {
        HAL_Delay(1);
    }
    radioSetInterrupts(0);

    state_ = 0;
    return 0;
}

//------------------------------------------------------------------------------
void radioSetInterrupts(uint16_t flags)
{
    uint8_t flags8[2];
    flags8[0] = flags & 0xFF;
    flags8[1] = (flags >> 8);

    radioWrite(RFM22_INTERRUPT_ENABLE1, flags8[0]);
    radioWrite(RFM22_INTERRUPT_ENABLE2, flags8[1]);
}

//------------------------------------------------------------------------------
uint16_t radioGetInterrupts()
{
    uint16_t flags = 0;

    flags   = radioRead(RFM22_INTERRUPT_ENABLE2);
    flags <<= 8;
    flags  |= radioRead(RFM22_INTERRUPT_ENABLE1);

    return flags;
}

//------------------------------------------------------------------------------
uint16_t radioWaitInterrupt()
{
    while(!radioInterrupt());
    return radioStatus();
}

//------------------------------------------------------------------------------
unsigned char radioTransmitPacket(unsigned char *data, unsigned char size)
{
    unsigned char status = 0;
    UNUSED(status);
    
    radioWrite(RFM22_FUNCTION_CONTROL2, 0x01);  // clear FIFOs
    radioWrite(RFM22_FUNCTION_CONTROL2, 0x00);  // clear FIFOs (reset)
    
    radioWrite(RFM22_TRANSMIT_PACKET_LENGTH, size);
    radioWriteBurst(RFM22_FIFO_ACCESS, data, size);
    
    radioWrite(RFM22_INTERRUPT_ENABLE1, 0x04);	// enable packet sent interrupt
    
    status = radioRead(RFM22_INTERRUPT_STATUS1);
    status = radioRead(RFM22_INTERRUPT_STATUS2);
    
    radioWrite(RFM22_FUNCTION_CONTROL1, 0x09);
    
    while(!radioInterrupt());
    
    radioWrite(RFM22_FUNCTION_CONTROL1, 0x01);  
    
    return 0;  
}

//------------------------------------------------------------------------------
void radioReadyMode(void)
{
    unsigned char status = 0;
    UNUSED(status);

    radioWrite(RFM22_FUNCTION_CONTROL1, 0x01); // rxon

    status = radioRead(RFM22_INTERRUPT_STATUS1);
    status = radioRead(RFM22_INTERRUPT_STATUS2);

    radioWrite(RFM22_INTERRUPT_ENABLE1, 0x00);  // disable interrupts
    radioWrite(RFM22_INTERRUPT_ENABLE2, 0x00);  // disable interrupts
}

//------------------------------------------------------------------------------
void radioSleepMode(void)
{
    unsigned char status = 0;
    UNUSED(status);

    status = radioRead(RFM22_INTERRUPT_STATUS1);
    status = radioRead(RFM22_INTERRUPT_STATUS2);

    radioWrite(RFM22_FUNCTION_CONTROL1, 0x00); // rxon
}

//------------------------------------------------------------------------------
void radioRxMode(void)
{
    unsigned char status = 0;
    UNUSED(status);

    state_ = STATE_RX;

    radioWrite(RFM22_FUNCTION_CONTROL2, 0x03); // clear FIFOs
    radioWrite(RFM22_FUNCTION_CONTROL2, 0x10); // clear FIFOs (reset)
    
    status = radioStatus();
    radioSetInterrupts(RFM22_VALID_PACKET | RFM22_RX_ALMOST_FULL);
    radioWrite(RFM22_FUNCTION_CONTROL1, 0x05); // rxon
}

//------------------------------------------------------------------------------
unsigned short radioStatus(void)
{
    unsigned short status = 0;
    
    status |=  radioRead(RFM22_INTERRUPT_STATUS1);
    status |= (radioRead(RFM22_INTERRUPT_STATUS2) << 8);
    
    return status;
}

//------------------------------------------------------------------------------
void radioSetPower(unsigned char power)
{
    power = power | 0x08;
    radioWrite(RFM22_TX_POWER, power & 0x0F);
}

//------------------------------------------------------------------------------
void radioSetRssiThreashold(uint8_t level)
{
    radioWrite(RFM22_RSSI_TRHESHOLD, level);
}

//------------------------------------------------------------------------------
void radioSetFrequency(uint32_t freq)
{
    uint8_t hbsel = 1;
    if(freq > 480000000UL)
    {
        hbsel = 2;
    }

    uint32_t band = (freq * 3) / (1000 * RFM22_XTAL * hbsel);
    uint32_t carrier = (freq / (10000 * hbsel)) * 64 - band * 64000;

    band = band + 40;

#ifdef DEBUG
    printf("Band: %02X", (uint8_t)(band & 0xFF));
    printf("Carrier: %04lX", carrier);
#endif

    radioWrite(RFM22_FREQUENCY_BAND, band & 0xFF);
    radioWrite(RFM22_CARRIER_FREQUENCY1, carrier >> 8);
    radioWrite(RFM22_CARRIER_FREQUENCY0, carrier & 0xFF);
}

//------------------------------------------------------------------------------
unsigned char radioGetPower(void)
{
    return (radioRead(RFM22_TX_POWER) & 0x07);
}

//------------------------------------------------------------------------------
void radioSetHeader(uint32_t dest)
{
    radioWriteBurst(RFM22_TRANSMIT_HEADER3, (uint8_t *)&dest, 4);
}

//------------------------------------------------------------------------------
void radioSetRxAddress(uint32_t address, uint32_t mask)
{
    radioWriteBurst(RFM22_CHECK_HEADER3, (unsigned char *)&address, 4);
    radioWriteBurst(RFM22_HEADER_ENABLE3, (unsigned char *)&mask, 4);
}

//------------------------------------------------------------------------------
unsigned char radioReceivePacket(unsigned char *data, uint16_t max_len)
{
    unsigned char len = radioRead(RFM22_RECEIVED_PACKET_LENGTH) + 5;

    if(len > max_len)
    {
        radioReadBurst(RFM22_FIFO_ACCESS, data, max_len);

        radioWrite(RFM22_FUNCTION_CONTROL2, 0x02);  // clear FIFOs
        radioWrite(RFM22_FUNCTION_CONTROL2, 0x00);  // clear FIFOs (reset)
    }
    else
    {
        radioReadBurst(RFM22_FIFO_ACCESS, data, len);
    }

    return len;
}

//------------------------------------------------------------------------------
uint8_t radioReceive(uint8_t *data, uint16_t max_len)
{
    uint8_t len = 0;

    if(rx_valid_)
    {
        len = rx_count_;
        memcpy(data, rx_buffer_, len);
        rx_valid_ = 0;
        rx_count_ = 0;
    }

    return len;
}
