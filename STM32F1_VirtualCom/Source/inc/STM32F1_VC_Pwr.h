/**
 * @file    STM32F1_VC_Pwr.h
 * @author  MSH RF/ART Team IMS-Systems Lab
 * @version V1.0.0
 * @date    August 4, 2011
 * @brief   Connection/disconnection & power management header.
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
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F1_VC_PWR_H
#define __STM32F1_VC_PWR_H


#ifdef __cplusplus
extern "C" {
#endif

#include "usb_lib.h"

/** @defgroup STM32F1_Virtual_Com    STM32F1 Virtual Com
 * @{
 */


/** @defgroup STM32F1_VC_Power   STM32F1 VC Power
 * @brief Connection/disconnection & power management module for Virtual COM Port Device.
 * @details See the file <i>@ref STM32F1_VC_Pwr.h</i> for more details.
 * @{
 */


/** @defgroup STM32F1_VC_Power_Exported_Types    STM32F1 VC Power Exported Types
 * @{
 */
  
/**
* @brief RESUME STATE Enumerative Types
*/    
typedef enum _RESUME_STATE
{
  RESUME_EXTERNAL,      /*!< External */
  RESUME_INTERNAL,      /*!< Internal */
  RESUME_LATER,         /*!< Later */
  RESUME_WAIT,          /*!< Wait */
  RESUME_START,         /*!< Start */
  RESUME_ON,            /*!< On */
  RESUME_OFF,           /*!< Off */
  RESUME_ESOF           /*!< ESOF */
} RESUME_STATE;

/**
* @brief DEVICE STATE Enumerative Types
*/  
typedef enum _DEVICE_STATE
{
  UNCONNECTED,          /*!< Unconnected */
  ATTACHED,             /*!< Atatched */
  POWERED,              /*!< Powered */
  SUSPENDED,            /*!< Supended */
  ADDRESSED,            /*!< Addressed */
  CONFIGURED            /*!< Configured */
} DEVICE_STATE;

/**
 * @}
 */


/** @defgroup STM32F1_VC_Power_Exported_Constants    STM32F1 VC Power Exported Constants
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_Power_Exported_Macros   STM32F1 VC Power Exported Macros
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_Power_External_Variables    STM32F1 VC Power External Variables
 * @{
*/
extern  __IO uint32_t bDeviceState; /* USB device status */
extern __IO bool fSuspendEnabled;  /* true when suspend is possible */

/**
 * @}
 */


/** @defgroup STM32F1_VC_Power_Exported_Functions    STM32F1 VC Power Exported Functions
 * @{
 */

void Suspend(void);
void Resume_Init(void);
void Resume(RESUME_STATE eResumeSetVal);
RESULT PowerOn(void);
RESULT PowerOff(void);

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
