/**
  ******************************************************************************
  * @file    iNEMO_usb_common.c
  * @author  ART Team IMS-Systems Lab
  * @version V3.2.1
  * @date    09/20/2010
  * @brief   This file provides
  *            - set of firmware functions to manage some common iNEMO USB Functionality
  *
  * @details
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
  * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
  * IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "usb_lib.h"
#include "iNEMO_DFU_Pwr.h"
#include "iNEMO_DFU_General.h"
#include "iNEMO_DFU_Mal.h"


/** @addtogroup iNEMO_DFU_Program
  * @{
  */

/** @addtogroup Common
  * @{
  */

/** @addtogroup iNEMO_USB_Common
  * @{
  */

/** @defgroup iNEMO_USB_Common_Functions
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void prviNemoDFUIntToUnicode(uint32_t lValue , uint8_t* pcBuffer , uint8_t cLength);
/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Configures main system clocks, USB clock, USB interrupt, GPIO pin for USB pull-up if defined
 *         and the Medium Interface as well as the USB peripheral.
 * @param  None
 * @retval None
 */
void iNemoDFUInit(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Unlock the internal flash */
  FLASH_Unlock();

  /* Enable "DISCONNECT" GPIO clock */
  RCC_APB2PeriphClockCmd(iNEMO_RCC_APB2Periph_GPIO_DISCONNECT, ENABLE);

  /* Configure USB pull-up */
  GPIO_InitStructure.GPIO_Pin = iNEMO_USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(iNEMO_USB_DISCONNECT, &GPIO_InitStructure);
  
  /* Init the media interface */
  MAL_Init();

  /*Enable the USB Cable*/
  iNemoDFUCableConfig(ENABLE);
  
  /* Select USBCLK source */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);

  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
  
  /* Configure USB interrupts */
  NVIC_InitStructure.NVIC_IRQChannel = iNEMO_USB_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = iNEMO_USB_PREEMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = iNEMO_USB_SUB_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* USB configuration */
  USB_Init();
}


/**
  * @brief  Power-off system clocks and power while entering suspend mode
  * @param  None
  * @retval None
  */
void iNemoDFUEnterLowPowerMode(void)
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;
}


/**
  * @brief  Restores system clocks and power while exiting suspend mode
  * @param  None
  * @retval None
  */
void iNemoDFULeaveLowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }
}

/**
  * @brief  Software Connection/Disconnection of USB Cable
  * @param  NewState: new state
  *   This parameter can be one of following parameters:
  *     @arg ENABLE
  *     @arg DISABLE
  * @retval None
  */
void iNemoDFUCableConfig(FunctionalState NewState)
{

  if (NewState != DISABLE)
  {
    GPIO_SetBits(iNEMO_USB_DISCONNECT, iNEMO_USB_DISCONNECT_PIN);
  }
  else
  {
    GPIO_ResetBits(iNEMO_USB_DISCONNECT, iNEMO_USB_DISCONNECT_PIN);
  }

}


/**
  * @brief  Reset the device
  * @param  None
  * @retval None
  */
void iNemoDFUResetDevice(void)
{
//  Leave_DFU_SW_Mode();
  iNemoDFUCableConfig(DISABLE);
  NVIC_SystemReset();
}



/**
  * @brief  Create the serial number string descriptor for DFU
  * @param  none.
  * @retval None
  */
void iNemoDFUGetSerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(__IO uint32_t*)(0x1FFFF7E8);
  Device_Serial1 = *(__IO uint32_t*)(0x1FFFF7EC);
  Device_Serial2 = *(__IO uint32_t*)(0x1FFFF7F0);

  Device_Serial0 += Device_Serial2;

  if (Device_Serial0 != 0)
  {
    prviNemoDFUIntToUnicode(Device_Serial0, &DFU_StringSerial[2] , 8);
    prviNemoDFUIntToUnicode(Device_Serial1, &DFU_StringSerial[18], 4);
  }
}


/**
 * @brief  Convert Hex 32Bits value into char.
 * @param  lValue: the HEX words to convert
 * @param  pcBuffer: the buffer in whic put the converted values
 * @param  cLength: the number of byte to convert
 * @retval None
 */
void prviNemoDFUIntToUnicode(uint32_t lValue , uint8_t* pcBuffer , uint8_t cLength)
{
  uint8_t idx = 0;

  for( idx = 0 ; idx < cLength ; idx ++)
  {
    if( ((lValue >> 28)) < 0xA )
    {
      pcBuffer[ 2* idx] = (lValue >> 28) + '0';
    }
    else
    {
      pcBuffer[2* idx] = (lValue >> 28) + 'A' - 10;
    }

    lValue = lValue << 4;

    pcBuffer[ 2* idx + 1] = 0;
  }

}

/**
  * @}
  */ /* end of group iNEMO_USB_Common_Functions */

/**
  * @}
  */ /* end of group iNEMO_USB_Common */

/**
  * @}
  */ /* end of group Common */

/**
  * @}
  */ /* end of group iNEMO_DFU_Program */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
