/**
 * @file    STM32F1_VC_General.h
 * @author  MSH RF/ART Team IMS-Systems Lab
 * @version V1.0.0
 * @date    August 4, 2011
 * @brief   Header for STM32F1 Virtual Com Setup & API.
 * @details
 *
 * This module provides API to use the Virtual Com.
 * An initialization function has to be called at the beginning of each program
 * in order to open the communication stream with the serial client.
 * One of the most used function is <i>@ref Stm32f1VCPrintf()</i>
 * based on an overriding of the standard C function <i>vsprintf()</i>.
 * It can be used to print a formatted string on the VCOM channel.
 * Moreover, there are other lower level API used to send a buffer or a char.
 * All the data to be sent are stored in a circular buffer of 512 bytes.
 * Every time the buffer has to be sent, the function <i>SdkEvalVCSendData()</i>
 * is required to be called.
 *
 * <b>Example:</b>
 * @code
 *
 *   uint8_t dummyNumber = 4;
 *
 *   ...
 *
 *   Stm32f1VCInit();
 *
 *   ...
 *
 *   Stm32f1VCPrintf("Hello world %d\n\r", dummyNumber);
 *   Stm32f1VCSendData();
 *
 *   ...
 *
 * @endcode
 *
 *
 * In order to receive data, two variables have been defined.
 * The first is the global input input buffer <i>@ref g_vectcVCRxBuffer</i> and
 * the global variable <i>@ref g_lVCNbBytesReceived</i>, which provides the number
 * of received bytes (it has to be reset or updated by the user every time the
 * received data are read).
 * The following example shows how to receive a string from a serial terminal (like
 * <i>hyperterminal</i> or <i>putty</i>) running on a PC.
 *
 * <b>Example:</b>
 * @code
 *  uint16_t n_from_vcom_data=0;
 *  uint8_t from_vcom[300];
 *
 *  while(1)
 *  {
 *    // control if the VCOM RX buffer is not empty
 *    if(g_lVCNbBytesReceived != 0){
 *      // copy each byte in an user-defined buffer
 *      for(uint16_t i=0 ; i<g_lVCNbBytesReceived ; i++)
 *        from_vcom[i+n_from_vcom_data]=g_vectcVCRxBuffer[i];
 *
 *      // compute the size of the received string
 *      n_from_vcom_data += g_lVCNbBytesReceived;
 *
 *      // free the VCOM RX buffer
 *      g_lVCNbBytesReceived=0;
 *
 *      // if '\r' has been received
 *      if(n_from_vcom_data && (char)from_vcom[n_from_vcom_data-1] == '\r'){
 *        // add the string termination chars
 *        from_vcom[n_from_vcom_data - 1] = '\0';
 *
 *        // exit the loop and return
 *        break;
 *      }
 *    }
 *  }
 *
 * @endcode
 *
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
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F1_VC_GENERAL_H
#define __STM32F1_VC_GENERAL_H



#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "STM32F1_VC_Desc.h"
#include "STM32F1_VC_Istr.h"


#ifdef __cplusplus
extern "C" {
#endif


/** @defgroup STM32F1_Virtual_Com  STM32F1 Virtual Com
 * @brief This module provides API to use the Virtual Com. In this way, the user can
 * communicate with the microcontroller using a common serial terminal.
 *
 * The <i>@ref STM32F1_VC_Conf.h</i> file must be edited in order to set the
 * product ID and the product string to the desired values. It's
 * advisable to locate this file in the same directory of the application
 * main.
 *
 * See the file <i>@ref STM32F1_VC_General.h</i> for more details and examples.
 *
 * @note The ST Virtual Com driver must be installed on the PC.
 *
 * @{
 */


/** @defgroup STM32F1_VC_General   STM32F1 VC General
 * @brief Main functions for Virtual COM Port Device.
 * @details See the file <i>@ref STM32F1_VC_General.h</i> for more details and examples.
 * @{
 */


/** @defgroup STM32F1_VC_General_Exported_Types    STM32F1 VC General Exported Types
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_General_Exported_Constants    STM32F1 VC General Exported Constants
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_General_Exported_Macros     STM32F1 VC General Exported Macros
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_General_Exported_Functions    STM32F1 VC General Exported Functions
 * @{
 */
void Stm32f1VCInit(void);
void Stm32f1VCResetDevice(void);
void Stm32f1VCCableConfig(FunctionalState NewState);
void Stm32f1VCEnterLowPowerMode(void);
void Stm32f1VCLeaveLowPowerMode(void);
void Stm32f1VCGetSerialNum(void);
void Stm32f1VCPrintf(const char *vectcStr,...);
void Stm32f1VCWriteTxBuffer(uint8_t* pcDataBuffer, uint16_t nNbBytes);
void Stm32f1VCResetCounter(void);
void Stm32f1VCSetCounter(uint32_t lInIndex, uint32_t lOutIndex);
void Stm32f1VCSendData(void);

/**
 * @}
 */


/**
 * @}
 */


/**
 * @}
 */



#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
