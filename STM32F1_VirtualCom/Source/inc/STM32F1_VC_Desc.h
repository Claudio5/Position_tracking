/**
  * @file    STM32F1_VC_Desc.h
  * @author  MCD Application Team & SystemsLab
  * @version V3.2.1
  * @date    09/20/2010
  * @brief   Descriptor Header for Virtual COM Port Device.
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F1_VC_DESC_H
#define __STM32F1_VC_DESC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "STM32F1_VC_Conf.h"

/** @defgroup STM32F1_Virtual_Com    STM32F1 Virtual Com
 * @{
 */


/** @defgroup STM32F1_VC_Descriptor    STM32F1 VC Descriptor
 * @brief Descriptor module for Virtual COM Port Device.
 * @details See the file <i>@ref STM32F1_VC_Desc.h</i> for more details.
 * @{
 */


/** @defgroup STM32F1_VC_Descriptor_Exported_Types   STM32F1 VC Descriptor Exported Types
 * @{
 */

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/

/**
* @brief USB Device Descriptor Type
*/  
#define USB_DEVICE_DESCRIPTOR_TYPE              0x01

/**
* @brief USB Configuration Descriptor Type
*/   
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
  
/**
* @brief USB String Descriptor Type
*/   
#define USB_STRING_DESCRIPTOR_TYPE              0x03
  
/**
* @brief USB Interface Descriptor Type
*/    
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
  
/**
* @brief USB Endpoint Descriptor Type
*/   
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05

/**
* @brief USB Virtual Com Port Data Size
*/   
#define VIRTUAL_COM_PORT_DATA_SIZE              64
  
/**
* @brief USB Virtual Com Port Int Size
*/   
#define VIRTUAL_COM_PORT_INT_SIZE               8

/**
* @brief USB Virtual Com Port Device Descriptor Size
*/   
#define VIRTUAL_COM_PORT_SIZ_DEVICE_DESC        18
  
/**
* @brief USB Virtual Com Port Configuration Descriptor Size
*/   
#define VIRTUAL_COM_PORT_SIZ_CONFIG_DESC        67
  
/**
* @brief USB Virtual Com Port String Descriptor Size
*/   
#define VIRTUAL_COM_PORT_SIZ_STRING_LANGID      4
  
/**
* @brief USB Virtual Com Port String Vendor Size
*/   
#define VIRTUAL_COM_PORT_SIZ_STRING_VENDOR      38
  
/**
* @brief USB Virtual Com Port String Product Size
*/   
#define VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT     (2+STRING_PRODUCT_SIZE)
  
/**
* @brief USB Virtual Com Port String Serial Size
*/   
#define VIRTUAL_COM_PORT_SIZ_STRING_SERIAL      26

/**
* @brief Standard Endpoint Descriptor Size
*/   
#define STANDARD_ENDPOINT_DESC_SIZE             0x09

/**
 * @}
 */


/** @defgroup STM32F1_VC_Descriptor_Exported_Constants   STM32F1 VC Descriptor Exported Constants
 * @{
 */

/**
* @brief   Virtual Com Port Device Descriptor
*/  
extern const uint8_t g_vectcVCPortDeviceDescriptor[VIRTUAL_COM_PORT_SIZ_DEVICE_DESC];

/**
* @brief   Virtual Com Port Configuration Descriptor
*/
extern const uint8_t g_vectcVCPortConfigDescriptor[VIRTUAL_COM_PORT_SIZ_CONFIG_DESC];

/**
* @brief USB Port String Descriptors
*/
extern const uint8_t g_vectcVCPortStringLangID[VIRTUAL_COM_PORT_SIZ_STRING_LANGID];

/**
* @brief USB Port String Vendor Descriptors
*/
extern const uint8_t g_vectcVCPortStringVendor[VIRTUAL_COM_PORT_SIZ_STRING_VENDOR];

/**
* @brief USB Port String Product Descriptors
*/
extern const uint8_t g_vectcVCPortStringProduct[VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT];

/**
* @brief USB Port String Serial Descriptors
*/
extern uint8_t g_vectcVCPortStringSerial[VIRTUAL_COM_PORT_SIZ_STRING_SERIAL];

/**
 * @}
 */


/** @defgroup STM32F1_VC_Descriptor_Exported_Macros      STM32F1 VC Descriptor Exported Macros
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_Descriptor_Exported_Functions   STM32F1 VC Descriptor Exported Functions
 * @{
 */

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
