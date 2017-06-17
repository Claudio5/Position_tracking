/**
  * @file    STM32F1_VC_Conf.h
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
#ifndef __STM32F1_VC_CONF_H
#define __STM32F1_VC_CONF_H

#ifdef __cplusplus
extern "C" {
#endif


/** @defgroup STM32F1_Virtual_Com    STM32F1 Virtual Com
 * @{
 */


/** @defgroup STM32F1_VC_Configuration    STM32F1 VC Configuration
 * @brief User defined parameters configuration module for Virtual COM Port Device.
 * @{
 */


/** @defgroup STM32F1_VC_Configuration_Exported_Types   STM32F1 VC Configuration Exported Types
 * @{
 */

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/

/**
 * @brief STM32 pin for SW USB cable connect/disconnect.
 */
#define STM32F1_USB_DISCONNECT                      GPIOA
/**
 * @brief STM32 port for SW USB cable connect/disconnect.
 */   
#define STM32F1_USB_DISCONNECT_PIN                  GPIO_Pin_9
/**
 * @brief STM32 clock port for SW USB cable connect/disconnect.
 */   
#define STM32F1_RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOA
  
/* Disconnect pin logic (comment this line if the cable connect logic is not inverted) */
//#define INVERTED_CONNECT

/**
 * @brief Memory address.
 */    
#define MASS_MEMORY_START                           0x04002000
/**
 * @brief Packet Size.
 */    
#define BULK_MAX_PACKET_SIZE                        0x00000040

/**
 * @brief Circular Buffer Size.
 */    
#define VC_TX_BUFFER_DATA_SIZE                      512

/**
 * @brief String Size Virtual Com name.
 */   
#define STRING_PRODUCT_SIZE                         68 
/**
 * @brief String for Virtual Com name.
 */    
#define STRING_PRODUCT_NAME             'S',0,'T',0,'E',0,'V',0,'A',0,'L',0,'-',0,'M',0,'K',0,'I',0,'1',0,'2',0,'1',0,'V',0,'1',0,' ',0,'i',0,'N',0,'E',0,'M',0,'O',0,' ',0,'D',0,'i',0,'s',0,'c',0,'o',0,'v',0,'e',0,'r',0,'y',0,'-',0,'M',0,'1',0
  
/* product ID: 600D */
/**
 * @brief LSB of Product ID.
 */    
#define PRODUCT_ID_LSB                              0x0D
/**
 * @brief MSB of Product ID.
 */   
#define PRODUCT_ID_MSB                              0x60
  
/**
 * @}
 */


/** @defgroup STM32F1_VC_Configuration_Exported_Constants   STM32F1 VC Configuration Exported Constants
 * @{
 */


/**
 * @}
 */


/** @defgroup STM32F1_VC_Configuration_Exported_Macros      STM32F1 VC Configuration Exported Macros
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_Configuration_Exported_Functions   STM32F1 VC Configuration Exported Functions
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
