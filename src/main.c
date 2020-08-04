#include "led.h"
#include "board.h"
#include "timer.h"
#include "protocol.h"
#include "datalink.h"
#include "rfm22b.h"
#include "usb/webusb.h"

#include "debug.h"

#include "tusb.h"

#include "stm32l0xx.h"

#include <stdio.h>
#include <stdint.h>

static USART_HandleTypeDef usart;
static Protocol protocol;

static void onReceive(Datalink *proto);
static void onFailure(Datalink *proto);

static RFM22BConfig radio =
{
    .spi = SPI1,
    .shdn = 2,
    .cs = 4,
    .irq = 3
};

static Datalink datalink =
{
    .src_addr = 0xFFFE,
    .retransmission = 3,
    .onReceive = onReceive,
    .onFailure = onFailure,
};

//------------------------------------------------------------------------------
void USB_IRQHandler(void)
{
    tud_int_handler(0);
}

//------------------------------------------------------------------------------
bool tud_vendor_control_request_cb(uint8_t rhport, tusb_control_request_t const * request)
{
    return webusbControlRequest(rhport, request);
}

//------------------------------------------------------------------------------
bool tud_vendor_control_complete_cb(uint8_t rhport, tusb_control_request_t const * request)
{
  (void) rhport;
  (void) request;

  return true;
}

//------------------------------------------------------------------------------
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
    /* User may add here some code to deal with this error */
    while(1)
    {
        ledToggle(LED1);
        HAL_Delay(100);
    }
}

//------------------------------------------------------------------------------
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  static RCC_CRSInitTypeDef RCC_CRSInitStruct;

  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable MSI Oscillator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_RCC_CRS_CLK_ENABLE();
  /* Default Synchro Signal division factor (not divided) */
   RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
   /* Set the SYNCSRC[1:0] bits according to CRS_Source value */
   RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
   /* HSI48 is synchronized with USB SOF at 1KHz rate */
   RCC_CRSInitStruct.ReloadValue =  __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
   RCC_CRSInitStruct.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;
   /* Set the TRIM[5:0] to the default value*/
   RCC_CRSInitStruct.HSI48CalibrationValue = 0x20;
   /* Start automatic synchronization */
   HAL_RCCEx_CRSConfig (&RCC_CRSInitStruct);

}

//------------------------------------------------------------------------------
void HAL_USART_MspInit(USART_HandleTypeDef *husart)
{
    __USART1_CLK_ENABLE();
}

//------------------------------------------------------------------------------
void HAL_MspInit(void)
{
    __GPIOA_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();

    boardInit();

#ifdef DEBUG
    usart.Instance = USART1;
    usart.Init.BaudRate = 115200;
    usart.Init.Mode = USART_MODE_TX;
    usart.Init.Parity = USART_PARITY_NONE;
    usart.Init.StopBits = USART_STOPBITS_1;
    HAL_USART_Init(&usart);
#endif
}

//------------------------------------------------------------------------------
static void onReceive(Datalink *proto)
{
    debug("Receive");
}

//------------------------------------------------------------------------------
static void onFailure(Datalink *proto)
{

}

//------------------------------------------------------------------------------
static void onFrameReceived(Protocol *proto, UsbFrame *frame)
{
    switch(frame->command)
    {
        case CMD_GET_VERSION:
            break;

        case CMD_READ_COUNTER:
            break;

        case CMD_RESET_COUNTER:
            break;

        case CMD_RESET:
            NVIC_SystemReset();
            break;

        default:
        {
            debug("Unknown Command: %d", frame->command);

            UsbFrame *resp = protocolAllocFrame(proto);
            usbFrameInit(resp, frame->command, FRAME_FLAG_NAK);
            protocolSend(proto, resp);
        }
    }
}

//------------------------------------------------------------------------------
void tud_cdc_rx_cb(uint8_t itf)
{
    (void)itf;
}

//------------------------------------------------------------------------------
int main(void)
{
    Timer timer;

    SystemClock_Config();
    HAL_Init();

    radioInit(&radio);
    radioInitRegs(0);

    datalinkInit(&datalink);

    protocolInit(&protocol);
    webusbInit(&protocol);

    protocolSetCallback(&protocol, onFrameReceived);

    info("RFUSB started");

    __HAL_RCC_USB_CLK_ENABLE();
    NVIC_SetPriority(USB_IRQn, 4);
    tusb_init();
    dcd_int_enable(0);

    timerSet(&timer, 500);
    while(1)
    {
        if(timerExpired(&timer))
        {
            timerRestart(&timer);
            ledToggle(LED0);

            info("Version: %x", radioRead(RFM22_DEVICE_VERSION));
        }

        tud_task();

        webusbService();
        protocolService(&protocol);
        datalinkService(&datalink);
    }
}
