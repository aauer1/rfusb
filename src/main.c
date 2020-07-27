#include "led.h"
#include "board.h"
#include "timer.h"
#include "protocol.h"
#include "usb/webusb.h"

#include "debug.h"

#include "tusb.h"

#include "stm32l0xx.h"

#include <stdio.h>
#include <stdint.h>

static USART_HandleTypeDef usart;
static Protocol protocol;

//------------------------------------------------------------------------------
void OTG_FS_IRQHandler(void)
{
    dcd_int_handler(0);
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

  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable MSI Oscillator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
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

  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

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
static void onFrameReceived(Protocol *proto, Frame *frame)
{
    switch(frame->command)
    {
        case CMD_ERASE_APPLICATION:
            break;

        case CMD_FIRMWARE_UPDATE:
            break;

        case CMD_RESET:
            NVIC_SystemReset();
            break;

        default:
        {
            debug("Unknown Command: %d", frame->command);

            Frame *resp = protocolAllocFrame(proto);
            frameInit(resp, frame->command, FLAG_NAK);
            protocolSend(proto, resp);
        }
    }
}

//------------------------------------------------------------------------------
int main(void)
{
    Timer timer;

    SystemClock_Config();
    HAL_Init();

    protocolInit(&protocol);
    webusbInit(&protocol);

    protocolSetCallback(&protocol, onFrameReceived);

    info("RFUSB started");
/*
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    NVIC_SetPriority(OTG_FS_IRQn, 10);
    USB_OTG_FS->GCCFG |= USB_OTG_GCCFG_NOVBUSSENS | USB_OTG_GCCFG_PWRDWN;
    tusb_init();
*/
    timerSet(&timer, 200);
    while(1)
    {
        if(timerExpired(&timer))
        {
            timerRestart(&timer);
            ledToggle(LED0);
        }
/*
        tud_task();
        webusbService();
        protocolService(&protocol);
*/
    }
}
