/**
  * @file    iNemo.h
  * @author  ART Team IMS-Systems Lab
  * @version V2.3.0
  * @date    01 February 2013
  * @brief   Header for iNEMO.c
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



#ifndef __INEMO_H_
#define __INEMO_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup iNEMO_Discovery_System
 * @{
 */
  
/**
 * @defgroup iNEMO_Core                           iNemo Core
 * @{
 */
  
/**
 * @brief Option Byte address from where read and write value to enter or leave DFU mode 
 */
#define OPTION_ADDRESS                           ((uint32_t)0x1FFFF804)

  
/**
 * @brief Task priorities base.
 */
#define IN_TASK_PRIORITY			( tskIDLE_PRIORITY + 1 )

  
/**
 * @brief Stack dimension for the iNemo Tasks.
 */
#define IN_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE + 200)

  
/**
 * @brief The structure that contains all the commands available in the GUI protocol.
 * @details An array of this type is guessed to be instanced in order to map the association between
 *          the command id and the handlers. 
 */
typedef struct
{
  uint8_t ucFrameId;                    /*!< Frame ID */
  uint8_t ucFrameLength;                /*!< Frame Length */
  void (*FrameHandler)(uint8_t* pcUsbFrame);    /*!< Payload */
}FrameMap;


/**
 * @defgroup iNEMO_Core_function             iNemo Core Functions
 * @{
 */
void iNemoInitTask(void *pvParameters);
void iNemoStartTask(void *pvParameters);
void iNemoTimerConfig(float fFrequency);

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
