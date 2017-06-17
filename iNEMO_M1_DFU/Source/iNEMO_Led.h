/**
  * @file    iNEMO_Led.h
  * @author  ART Team IMS-Systems Lab
  * @version V2.3.0
  * @date    02 September 2011
  * @brief   iNEMO Led Hardware Configuration & Setup
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
#ifndef __INEMO_LED_H
#define __INEMO_LED_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

#ifdef __cplusplus
 extern "C" {
#endif

/** @defgroup iNEMO_Led
  * @{
  */

/** @defgroup iNEMO_Led_Prototypes
  * @{
  */


typedef enum 
{
  LED1 = 0
} Led_TypeDef;


/* Define the Led Pin */  
#define iNEMO_LEDn                             1

#define iNEMO_LED1_PIN                         GPIO_Pin_1
#define iNEMO_LED1_GPIO_PORT                   GPIOA
#define iNEMO_LED1_GPIO_CLK                    RCC_APB2Periph_GPIOA


void iNEMO_Led_Init(Led_TypeDef Led);
void iNEMO_Led_On(Led_TypeDef Led);
void iNEMO_Led_Off(Led_TypeDef Led);
void iNEMO_Led_Toggle(Led_TypeDef Led);

/**
  * @}
  */ /* end of group iNEMO_Led_Prototypes */

/**
  * @}
  */ /* end of group iNEMO_Led */

#ifdef __cplusplus
}
#endif

#endif /* __INEMO_LED_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/