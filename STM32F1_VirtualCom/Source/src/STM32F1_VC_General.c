/**
 * @file     STM32F1_VC_General.c
 * @author   MSH RF/ART Team IMS-Systems Lab
 * @version  V1.0.0
 * @date     August 4, 2011
 * @brief    STM32F1 Virtual Com Setup & API.
 * @details
 *
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
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 *
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

#include "usb_lib.h"
#include "usb_conf.h"
#include "STM32F1_VC_General.h"
#include "STM32F1_VC_Desc.h"
#include "STM32F1_VC_Istr.h"
#include "STM32F1_VC_Prop.h"
#include "STM32F1_VC_Pwr.h"
#include "STM32F1_VC_Conf.h"

#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>

/** @addtogroup STM32F1_Virtual_Com
 * @{
 */


/** @addtogroup STM32F1_VC_General
 * @{
 */


/** @defgroup STM32F1_VC_General_Private_TypesDefinitions    STM32F1 VC General Private TypesDefinitions
 * @{
 */


/**
 * @}
 */


/** @defgroup STM32F1_VC_General_Private_Macros    STM32F1 VC General Private Macros
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_General_Private_Variables     STM32F1 VC General Private Variables
 * @{
 */

/**
* @brief Volatile Flash Status variable
*/ 
__IO FLASH_Status FLASHStatus = FLASH_COMPLETE;

/**
* @brief Buffer where store data to be sent
*/ 
static uint8_t  s_vectcVCTxBuffer [VC_TX_BUFFER_DATA_SIZE];

/**
* @brief In Index of TX Buffer
*/ 
static uint32_t s_lVCTxBufferInIndex = 0;

/**
* @brief Out Index of TX Buffer
*/ 
static uint32_t s_lVCTxBufferOutIndex = 0;

/**
* @brief Length of data to be sent
*/ 
static uint32_t s_lVCTxDataLength  = 0;

/**
* @brief Linecoding variable setting
*/ 
extern LineCoding xLinecoding;



/**
 * @}
 */


/** @defgroup STM32F1_VC_General_Private_FunctionPrototypes      STM32F1 VC General Private FunctionPrototypes
 * @{
 */
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);

/**
 * @}
 */


/** @defgroup STM32F1_EVAL_VC_General_Private_Functions     STM32F1 VC General Private Functions
 * @{
 */

/**
 * @brief  Configures main system clocks, USB clock, USB interrupt and GPIO pin for USB pull-up if defined.
 * @retval None.
 */
void Stm32f1VCInit(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* Enable "DISCONNECT" GPIO clock */
  RCC_APB2PeriphClockCmd(STM32F1_RCC_APB2Periph_GPIO_DISCONNECT, ENABLE);

  /* Configure USB pull-up */
  GPIO_InitStructure.GPIO_Pin = STM32F1_USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(STM32F1_USB_DISCONNECT, &GPIO_InitStructure);

  Stm32f1VCCableConfig(ENABLE);

  /* USBCLK = PLLCLK / 1.5 */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  /* Enable USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);

  /* Configures USB interrupt */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0C;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Initializes USB system */
  USB_Init();

  while(bDeviceState != CONFIGURED);
  
}


/**
  * @brief  Software Connection/Disconnection of USB Cable
  * @param  NewState: new state
  *   This parameter can be one of following parameters:
  *     @arg ENABLE
  *     @arg DISABLE
  * @retval None
  */
void Stm32f1VCCableConfig (FunctionalState NewState)
{
#ifndef INVERTED_CONNECT
  if (NewState != DISABLE)
#else
  if (NewState != ENABLE) 
#endif
  {
    GPIO_SetBits(STM32F1_USB_DISCONNECT, STM32F1_USB_DISCONNECT_PIN);
  }
  else
  {
    GPIO_ResetBits(STM32F1_USB_DISCONNECT, STM32F1_USB_DISCONNECT_PIN);
  }

}



/**
  * @brief  Reset the device
  * @retval None
  */
void Stm32f1VCResetDevice(void)
{
  Stm32f1VCCableConfig(DISABLE);
  NVIC_SystemReset();
}



/**
  * @brief  Power-off system clocks and power while entering suspend mode.
  * @retval None
  */
void Stm32f1VCEnterLowPowerMode(void)
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;
}


/**
  * @brief  Restores system clocks and power while exiting suspend mode.
  * @retval None
  */

void Stm32f1VCLeaveLowPowerMode(void)
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
  * @brief  Create the serial number string descriptor.
  * @retval None
  */

void Stm32f1VCGetSerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(__IO uint32_t*)(0x1FFFF7E8);
  Device_Serial1 = *(__IO uint32_t*)(0x1FFFF7EC);
  Device_Serial2 = *(__IO uint32_t*)(0x1FFFF7F0);

  Device_Serial0 += Device_Serial2;

  if (Device_Serial0 != 0)
  {
    IntToUnicode (Device_Serial0, &g_vectcVCPortStringSerial[2] , 8);
    IntToUnicode (Device_Serial1, &g_vectcVCPortStringSerial[18], 4);
  }
}


/**
  * @brief  Write n characters on the serial peripheral.
  * @param  vectcStr: data to be written.
  * @retval None
  */
void Stm32f1VCPrintf(const char *vectcStr,...)
{
  char buffer[256];
  uint8_t index = 0;
  va_list arglist;

  va_start(arglist,vectcStr);
  vsprintf(buffer,vectcStr,arglist);
  va_end(arglist);

  /* Evaluates the size of string */
  for(index=0 ; buffer[index] != '\0' ; index++);

  /* Copy in the circular TX buffer */
  Stm32f1VCWriteTxBuffer((unsigned char*)buffer, index);

}


/**
 * @brief  Puts in the circular buffer the data to transfer through USB.
 * @param  pcDataBuffer pointer to the buffer containing the bytes to put in the circular buffer.
 * @param  nNbBytes the number of bytes to write in the buffer.
 * @retval None.
 */
void Stm32f1VCWriteTxBuffer(uint8_t* pcDataBuffer, uint16_t nNbBytes)
{
  uint32_t inIndex = s_lVCTxBufferInIndex;

  for(uint16_t i=0; i<nNbBytes; i++)
  {
    s_vectcVCTxBuffer[inIndex] = *pcDataBuffer;
    inIndex++;
    /* To avoid buffer overflow */
    if(inIndex == VC_TX_BUFFER_DATA_SIZE)
    {
      inIndex = 0;
    }
    pcDataBuffer++;
  }
   s_lVCTxBufferInIndex = inIndex;
}


/**
 * @brief  Resets the static variable only for test purpose.
 * @retval None.
 */
void Stm32f1VCResetCounter(void)
{
  s_lVCTxBufferInIndex = 0;
  s_lVCTxBufferOutIndex = 0;
}


/**
 * @brief  Sets the static variable only for test purpose.
 * @retval None.
 */
void Stm32f1VCSetCounter(uint32_t lInIndex, uint32_t lOutIndex)
{
  s_lVCTxBufferInIndex = lInIndex;
  s_lVCTxBufferOutIndex = lOutIndex;
}


/**
 * @brief  Transfers available data through USB.
 * @retval None.
 */
void Stm32f1VCSendData(void)
{
  uint16_t USB_Tx_ptr;
  uint16_t USB_Tx_length;
  bool DataToTransfer = FALSE;

  uint32_t lVCTxBufferInIndex=s_lVCTxBufferInIndex;
  
  if(s_lVCTxBufferOutIndex != lVCTxBufferInIndex)
  {
    DataToTransfer = TRUE;
    while(DataToTransfer == TRUE)
    {
      if(s_lVCTxBufferOutIndex == lVCTxBufferInIndex)
      {
        DataToTransfer = FALSE;
      }
      if(s_lVCTxBufferOutIndex > lVCTxBufferInIndex) /* rollback */
      {
        s_lVCTxDataLength = VC_TX_BUFFER_DATA_SIZE - s_lVCTxBufferOutIndex;
      }
      else
      {
        s_lVCTxDataLength = lVCTxBufferInIndex - s_lVCTxBufferOutIndex;
      }
      if (s_lVCTxDataLength < VIRTUAL_COM_PORT_DATA_SIZE)
      {
        USB_Tx_ptr = s_lVCTxBufferOutIndex;
        USB_Tx_length = s_lVCTxDataLength;

        s_lVCTxBufferOutIndex += s_lVCTxDataLength;
        s_lVCTxDataLength = 0;
        if (s_lVCTxBufferOutIndex == VC_TX_BUFFER_DATA_SIZE)
        {
          s_lVCTxBufferOutIndex = 0;
        }
        else
        {
          DataToTransfer = FALSE;
        }
      }
      else
      {
        USB_Tx_ptr = s_lVCTxBufferOutIndex;
        USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;

        s_lVCTxBufferOutIndex += VIRTUAL_COM_PORT_DATA_SIZE;
        s_lVCTxDataLength -= VIRTUAL_COM_PORT_DATA_SIZE;
      }
      while (GetEPTxStatus(ENDP1) == EP_TX_VALID);
      UserToPMABufferCopy(&s_vectcVCTxBuffer[USB_Tx_ptr], ENDP1_TXADDR, USB_Tx_length);
      SetEPTxCount(ENDP1, USB_Tx_length);
      SetEPTxValid(ENDP1);
    }
  }

}


/**
  * @brief  Convert Hex 32Bits value into char
  * @retval None
  */
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;

  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10;
    }

    value = value << 4;

    pbuf[ 2* idx + 1] = 0;
  }
}


/**
 * @}
 */


/**
 * @}
 */


/**
 * @}
 */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
