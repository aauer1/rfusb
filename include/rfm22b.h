#ifndef RFM22B_H
#define RFM22B_H

#include "board.h"

#include <stdint.h>

#define RFM22_XTAL					30000UL

#define RFM22_CRC_ERROR             0x0001
#define RFM22_VALID_PACKET          0x0002
#define RFM22_PACKET_SENT           0x0004
#define RFM22_EXTERNAL_INT          0x0008
#define RFM22_RX_ALMOST_FULL        0x0010
#define RFM22_TX_ALMOST_EMPTY       0x0020
#define RFM22_TX_ALMOST_FULL        0x0040
#define RFM22_FIFO_ERROR            0x0080
#define RFM22_POR_RESET             0x0100
#define RFM22_MODUL_READY           0x0200
#define RFM22_LOW_BATTERY           0x0400
#define RFM22_WAKEUP_TIMER          0x0800
#define RFM22_RSSI_INDICATOR        0x1000
#define RFM22_INVALID_PREAMBLE      0x2000
#define RFM22_VALID_PREAMBLE        0x4000
#define RFM22_SYNC_WORD             0x8000

enum RFM22_Registers
{
    RFM22_DEVICE_TYPE = 0,
    RFM22_DEVICE_VERSION,
    RFM22_DEVICE_STATUS,
    RFM22_INTERRUPT_STATUS1,
    RFM22_INTERRUPT_STATUS2,
    RFM22_INTERRUPT_ENABLE1,
    RFM22_INTERRUPT_ENABLE2,
    RFM22_FUNCTION_CONTROL1,
    RFM22_FUNCTION_CONTROL2,
    RFM22_CRYSTAL_LOAD,
    RFM22_MC_OUTPUT_CLOCK,
    RFM22_GPIO0,
    RFM22_GPIO1,
    RFM22_GPIO2,
    RFM22_IO_PORT_CONFIG,
    RFM22_ADC_CONFIG,
    RFM22_ADC_SENSOR_OFFSET,
    RFM22_ADC_VALUE,
    RFM22_TEMPERATUR_CONTROL,
    RFM22_TEMPERATUR_OFFSET,
    RFM22_WAKEUP_PERIOD1,
    RFM22_WAKEUP_PERIOD2,
    RFM22_WAKEUP_PERIOD3,
    RFM22_WAKEUP_VALUE1,
    RFM22_WAKEUP_VALUE2,
    RFM22_LOW_DUTY_CYCLE_DURATION,
    RFM22_LOW_BATTERY_THRESHOLD,
    RFM22_BATTERY_VOLTAGE,
    RFM22_IF_BANDWIDTH,
    RFM22_AFC_LOOP_GEARSHIFT,
    RFM22_AFC_TIMING_CONTROL,
    RFM22_CLOCK_RECOVERY_GEARSHIFT,
    RFM22_CLOCK_RECOVERY_OVERSAMPLING,
    RFM22_CLOCK_RECOVERY_OFFSET2,
    RFM22_CLOCK_RECOVERY_OFFSET1,
    RFM22_CLOCK_RECOVERY_OFFSET0,
    RFM22_CLOCK_RECOVERY_LOOP_GAIN1,
    RFM22_CLOCK_RECOVERY_LOOP_GAIN0,
    RFM22_RSSI,
    RFM22_RSSI_TRHESHOLD,
    RFM22_ANTENNA_DIVERSITY1,
    RFM22_ANTENNA_DIVERSITY2,
    
    RFM22_AFC_LIMITER = 0x2A,
    RFM22_AFC_VALUE,

    RFM22_DATA_ACCESS_CONTROL = 0x30,
    RFM22_EZMAC_STATUS,
    RFM22_HEADER_CONTROL1,
    RFM22_HEADER_CONTROL2,
    RFM22_PREAMBLE_LENGTH,
    RFM22_PREAMBLE_CONTROL1,
    RFM22_SYNC_WORD3,
    RFM22_SYNC_WORD2,
    RFM22_SYNC_WORD1,
    RFM22_SYNC_WORD0,
    RFM22_TRANSMIT_HEADER3,
    RFM22_TRANSMIT_HEADER2,
    RFM22_TRANSMIT_HEADER1,
    RFM22_TRANSMIT_HEADER0,
    RFM22_TRANSMIT_PACKET_LENGTH,
    RFM22_CHECK_HEADER3,
    RFM22_CHECK_HEADER2,
    RFM22_CHECK_HEADER1,
    RFM22_CHECK_HEADER0,
    RFM22_HEADER_ENABLE3,
    RFM22_HEADER_ENABLE2,
    RFM22_HEADER_ENABLE1,
    RFM22_HEADER_ENABLE0,
    RFM22_RECEIVED_HEADER3,
    RFM22_RECEIVED_HEADER2,
    RFM22_RECEIVED_HEADER1,
    RFM22_RECEIVED_HEADER0,
    RFM22_RECEIVED_PACKET_LENGTH,

    RFM22_ANALOG_TEST_BUS = 0x50,
    RFM22_DIGITAL_TEST_BUS,
    RFM22_TX_RAMP_CONTROL,
    RFM22_PLL_TUNE_TIME,

    RFM22_CALIBRATION_CONTROL = 0x55,
    RFM22_MODEM_TEST,
    RFM22_CHARGEPUMP_TEST,
    RFM22_CHARGEPUMP_CURRENT,
    RFM22_DIVIDER_CURRENT,
    RFM22_VCO_CURRENT,
    RFM22_VCO_CALIBRATION,
    RFM22_SYNTHESIZER_TEST,
    RFM22_BLOCK_ENABLE1,
    RFM22_BLOCK_ENABLE2,
    RFM22_BLOCK_ENABLE3,
    RFM22_CHANNEL_FILTER_ADDRESS,
    RFM22_CHANNEL_FILTER_VALUE,
    RFM22_CRYSTAL_OSCILLATOR,
    RFM22_RC_OSCILLATOR_COARSE_CALIBRATION,
    RFM22_RC_OSCILLATOR_FINE_CALIBRATION,
    RFM22_LDO_CONTROL,
    RFM22_LDO_LEVEL,
    RFM22_ADC_TUNING1,
    RFM22_ADC_TUNING2,
    RFM22_AGC_OVERRIDE1,
    RFM22_AGC_OVERRIDE2,
    RFM22_GFSK_FIR_FILTER_ADDRESS,
    RFM22_GFSK_FIR_FILTER_VALUE,
    RFM22_TX_POWER,
    RFM22_TX_DATA_RATE1,
    RFM22_TX_DATA_RATE0,
    RFM22_MODULATION_CONTROL1,
    RFM22_MODULATION_CONTROL2,
    RFM22_FREQUENCY_DEVIATION,
    RFM22_FREQUENCY_OFFSET1,
    RFM22_FREQUENCY_OFFSET2,
    RFM22_FREQUENCY_BAND,
    RFM22_CARRIER_FREQUENCY1,
    RFM22_CARRIER_FREQUENCY0,
    
    RFM22_FREQUENCY_HOPPING_CHANNEL = 0x79,
    RFM22_FREQUENCY_HOPPING_STEP,
    
    RFM22_TX_FIFO_CONTROL1 = 0x7C,
    RFM22_TX_FIFO_CONTROL2,
    RFM22_RX_FIFO_CONTROL,
    RFM22_FIFO_ACCESS,
};

typedef struct RFM22B_RegEntry_ RFM22B_RegEntry;
typedef struct RFM22BConfig_ RFM22BConfig;

struct RFM22B_RegEntry_
{
    int8_t addr;
    uint8_t value;
};

struct RFM22BConfig_
{
    SPI_TypeDef *spi;

    uint8_t shdn;
    uint8_t cs;
    uint8_t irq;

    // private:
    SPI_HandleTypeDef hspi;
};

void radioInit(RFM22BConfig *config);
void radioInitRegs(const RFM22B_RegEntry *regs);
void radioIrqHandler(uint16_t pin);
unsigned char radioInterrupt(void);
void radioShutdown(void);
void radioWakeup(void);
unsigned char radioRead(unsigned char addr);
void radioWrite(unsigned char addr, unsigned char value);
void radioReadBurst(unsigned char addr, unsigned char *data, unsigned char len);
void radioWriteBurst(unsigned char addr, unsigned char *data, unsigned char len);

uint8_t radioTransmit(uint8_t *data, uint16_t size);
void radioSetInterrupts(uint16_t flags);
uint16_t radioGetInterrupts();
uint16_t radioWaitInterrupt();

unsigned char radioTransmitPacket(unsigned char *data, unsigned char size);
void radioReadyMode(void);
void radioSleepMode(void);
void radioRxMode(void);
unsigned short radioStatus(void);
void radioSetPower(unsigned char power);
unsigned char radioGetPower(void);
void radioSetRssiThreashold(uint8_t level);
void radioSetFrequency(uint32_t freq);
void radioSetHeader(uint32_t dest);
void radioSetRxAddress(uint32_t address, uint32_t mask);
unsigned char radioReceivePacket(unsigned char *data, uint16_t max_len);

uint8_t radioReceive(uint8_t *data, uint16_t max_len);

#endif

