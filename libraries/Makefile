TARGET=libstm32l0.a
CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar

OBJDIR=obj

MCU=cortex-m0plus
DEVICE=STM32L082xx

###########################################

vpath %.c STM32L0xx_HAL_Driver/src

CFLAGS += -mlittle-endian -mthumb -mcpu=$(MCU) -march=armv6s-m
CFLAGS += -g -O2 -Wall -D$(DEVICE) -DUSE_HAL_DRIVER -fstack-usage
CFLAGS += -ffreestanding -nostdlib -ffunction-sections -fdata-sections -Wl,--gc-sections
CFLAGS += -I./ -ICMSIS/Include -ICMSIS/Device/ST/STM32L0xx/Include -ISTM32L0xx_HAL_Driver/inc

SRCS =  stm32l0xx_ll_spi.c \
        stm32l0xx_hal_msp_template.c \
        stm32l0xx_hal_crc.c \
        stm32l0xx_hal_pwr.c \
        stm32l0xx_hal_crc_ex.c \
        stm32l0xx_hal_irda.c \
        stm32l0xx_hal_uart_ex.c \
        stm32l0xx_hal_lptim.c \
        stm32l0xx_hal_i2s.c \
        stm32l0xx_hal_gpio.c \
        stm32l0xx_hal_tim_ex.c \
        stm32l0xx_ll_exti.c \
        stm32l0xx_ll_lpuart.c \
        stm32l0xx_hal_wwdg.c \
        stm32l0xx_hal_comp.c \
        stm32l0xx_hal_pcd_ex.c \
        stm32l0xx_hal_rcc.c \
        stm32l0xx_hal_rtc.c \
        stm32l0xx_hal_cryp_ex.c \
        stm32l0xx_ll_rtc.c \
        stm32l0xx_ll_i2c.c \
        stm32l0xx_hal_firewall.c \
        stm32l0xx_ll_gpio.c \
        stm32l0xx_hal_smartcard.c \
        stm32l0xx_hal_lcd.c \
        stm32l0xx_hal_flash_ex.c \
        stm32l0xx_ll_crs.c \
        stm32l0xx_hal_dma.c \
        stm32l0xx_hal_tim.c \
        stm32l0xx_hal_i2c_ex.c \
        stm32l0xx_hal_pcd.c \
        stm32l0xx_hal_flash.c \
        stm32l0xx_hal_uart.c \
        stm32l0xx_ll_adc.c \
        stm32l0xx_ll_tim.c \
        stm32l0xx_hal_rng.c \
        stm32l0xx_hal_comp_ex.c \
        stm32l0xx_hal_rcc_ex.c \
        stm32l0xx_ll_comp.c \
        stm32l0xx_hal_cortex.c \
        stm32l0xx_hal_i2c.c \
        stm32l0xx_ll_pwr.c \
        stm32l0xx_hal_pwr_ex.c \
        stm32l0xx_hal.c \
        stm32l0xx_hal_adc_ex.c \
        stm32l0xx_hal_iwdg.c \
        stm32l0xx_hal_adc.c \
        stm32l0xx_ll_crc.c \
        stm32l0xx_hal_dac_ex.c \
        stm32l0xx_hal_smbus.c \
        stm32l0xx_ll_utils.c \
        stm32l0xx_hal_tsc.c \
        stm32l0xx_hal_spi.c \
        stm32l0xx_ll_rng.c \
        stm32l0xx_hal_rtc_ex.c \
        stm32l0xx_hal_cryp.c \
        stm32l0xx_ll_lptim.c \
        stm32l0xx_hal_smartcard_ex.c \
        stm32l0xx_hal_usart.c \
        stm32l0xx_hal_dac.c \
        stm32l0xx_hal_flash_ramfunc.c \
        stm32l0xx_ll_usart.c \
        stm32l0xx_ll_dac.c \
        stm32l0xx_ll_dma.c \
        stm32l0xx_ll_rcc.c

OBJS = $(addprefix $(OBJDIR)/, $(SRCS:.c=.o))

.PHONY: all

all: init $(TARGET)

init:
	@if [ ! -d "obj/" ]; then mkdir obj; fi

$(OBJDIR)/%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $^

$(TARGET): $(OBJS)
	$(AR) -r $@ $(OBJS)

clean:
	rm -f obj/* $(TARGET)
