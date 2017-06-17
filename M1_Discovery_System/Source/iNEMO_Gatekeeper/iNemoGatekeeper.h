/**
 * @file    iNemoGatekeeper.h
 * @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
 * @version V1.0.1
 * @date    01 February 2013
 * @brief   Usb management module for iNemo.
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
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INEMOGATE_H_
#define INEMOGATE_H_


#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


#ifdef __cplusplus
 extern "C" {
#endif

   
/**
 * @addtogroup iNEMO_Discovery_System
 * @{
 */

/**
 * @defgroup iNemo_Gatekeeper                               iNemo Gatekeeper
 * @brief This module is responsible for the phisical host communication.<br>
 *        The gatekeeper is a task used to avoid race condition on the communication peripherial 
 *        (USB,RS232,RF,...) since it is the only one that accesses it.
 * @details It exports the API to send/receive buffers of data storing them in a FreeRTOS queue.
 *        The header file is indipendent from the communication peripherial. The details of the communication
 *        management are available in the .c file that implements these functions.
 * @{
 */


/**
 * @brief The value used to highlight an input frame
 */
#define INPUT_FRAME	1
   
/**
 * @brief The value used to highlight an output frame
 */
#define OUTPUT_FRAME	0


/**
 * @brief The structure type used to manage the usb frame
 */
typedef struct
{
  uint8_t uciNemoFrameLength;				/*!< The frame length. */
  uint8_t uciNemoFrame[100];                            /*!< The frame. */
  uint8_t uciNemoIOFrame;                               /*!< The I/O frame ID. */
}iNemoFrame;



/**
 * @defgroup iNEMO_Gatekeeper_Functions         iNemo USB Gatekeeper Functions
 * @{
 */

void iNemoGateKeeperTaskHwSetup(void);
void iNemoGateKeeperTaskCreate(void *pvParameters);
void iNemoGKSendData(uint8_t* ucFrame, uint8_t ucFrameLength);



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


 /******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/

