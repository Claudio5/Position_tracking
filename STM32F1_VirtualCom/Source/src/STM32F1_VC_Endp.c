/**
  * @file    STM32F1_VC_Endp.c
  * @author  MCD Application Team & SystemsLab
  * @version V3.2.1
  * @date    09/20/2010
  * @brief   Endpoint routines.
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
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */



/* Includes ------------------------------------------------------------------*/

#include "usb_lib.h"
#include "STM32F1_VC_Desc.h"


/** @addtogroup STM32F1_Virtual_Com
 * @{
 */


/** @addtogroup STM32F1_VC_Endpoint	STM32F1 VC Endpoint
 * @brief Implementation of endpoint routines for Virtual COM Port Device.
 * @details See the file <i>@ref STM32F1_VC_Endp.c</i> for more details.
 * @{
 */


/** @defgroup STM32F1_VC_Endpoint_Private_TypesDefinitions   STM32F1 VC Endpoint Private TypesDefinitions
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_Endpoint_Private_Defines    STM32F1 VC Endpoint Private Defines
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_Endpoint_Private_Macros   STM32F1 VC Endpoint Private Macros
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_Endpoint_Private_Variables    STM32F1 VC Endpoint Private Variables
 * @{
 */
/**
 * @brief RX Buffer
 */
uint8_t g_vectcVCRxBuffer[VIRTUAL_COM_PORT_DATA_SIZE];

/**
 * @brief Number of received bytes
 */
u32 g_lVCNbBytesReceived = 0;


/**
 * @}
 */


/** @defgroup STM32F1_VC_Endpoint_Private_FunctionPrototypes   STM32F1 VC Endpoint Private FunctionPrototypes
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_Endpoint_Private_Functions    STM32F1 VC Endpoint Private Functions
 * @{
 */

/**
 * @brief  Endpoint3_Output Callback.
 * @retval None
 */
void EP3_OUT_Callback(void)
{
  g_lVCNbBytesReceived = GetEPRxCount(ENDP3);
  PMAToUserBufferCopy(g_vectcVCRxBuffer, ENDP3_RXADDR, g_lVCNbBytesReceived);
  SetEPRxValid(ENDP3);

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

