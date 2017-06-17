/**
  * @file    iNemoLed.h
  * @author  ART Team IMS-Systems Lab
  * @version V2.3.0
  * @date    01 February 2013
  * @brief   iNEMO Led Hardware Configuration & Setup.
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
#ifndef __INEMO_LED_H
#define __INEMO_LED_H

#include "stm32f10x.h"

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @addtogroup iNEMO_Discovery_System
 * @{
 */
   
/** @defgroup iNemo_Led                             iNemo Led
 * @{
 */
   
/**
 * @defgroup iNEMO_Led_TypesDef                     iNemo Led Types Definition
 * @{
 */
/**
 * @brief  Enumerative type associated to LEDs.
 */    
typedef enum 
{
  LED1 = 0,
} Led_TypeDef;

/**
 * @}
 */



/**
 * @defgroup iNEMO_Led_Functions                     iNEMO Led Functions
 * @{
 */

void iNEMO_Led_Init(Led_TypeDef Led);
void iNEMO_Led_On(Led_TypeDef Led);
void iNEMO_Led_Off(Led_TypeDef Led);
void iNEMO_Led_Toggle(Led_TypeDef Led);

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
