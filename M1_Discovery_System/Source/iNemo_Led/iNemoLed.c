/**
  * @file    iNemoLed.c
  * @author  ART Team IMS-Systems Lab
  * @version V2.3.0
  * @date    01 February 2013
  * @brief   This file provides a set of functions to manage iNEMO Leds
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



/* Includes ------------------------------------------------------------------*/
#include "iNemoLed.h"
#include "iNemoConf.h"

/**
 * @addtogroup iNEMO_Discovery_System
 * @{
 */

/** @addtogroup iNemo_Led
 * @brief This module exports API to manage the iNemo LED.
 * @{
 */



/** @defgroup iNemo_Led_PrivateVariables              iNemo Led Private Variables
 * @{
 */
/**
 * @brief  Array for STM32 port associated to LEDs.
 */ 
GPIO_TypeDef* LED_PORT[iNEMO_LEDn] = {iNEMO_LED1_GPIO_PORT};
/**
 * @brief  Array for STM32 pin associated to LEDs.
 */ 
const uint16_t LED_PIN[iNEMO_LEDn] = {iNEMO_LED1_PIN};
/**
 * @brief  Array for STM32 port clock associated to LEDs.
 */ 
const uint32_t LED_CLK[iNEMO_LEDn] = {iNEMO_LED1_GPIO_CLK};


/** @defgroup iNEMO_Led_Functions               iNEMO Led Functions
 * @{
 */

/**
 * @brief  Configures LED GPIO.
 * @param  Led: Specifies the Led to be configured. 
 *   This parameter can be one of following parameters:
 *     @arg LED1
 * @retval None
 */
void iNEMO_Led_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_APB2PeriphClockCmd(LED_CLK[Led], ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = LED_PIN[Led];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(LED_PORT[Led], &GPIO_InitStructure);
}


/**
 * @brief  Turns selected LED On.
 * @param  Led: Specifies the Led to be set on. 
 *   This parameter can be one of following parameters:
 *     @arg LED1
 * @retval None
 */
void iNEMO_Led_On(Led_TypeDef Led)
{
  LED_PORT[Led]->BSRR = LED_PIN[Led];     
}


/**
 * @brief  Turns selected LED Off.
 * @param  Led: Specifies the Led to be set off. 
 *   This parameter can be one of following parameters:
 *    @arg LED1
 * @retval None
 */
void iNEMO_Led_Off(Led_TypeDef Led)
{
  LED_PORT[Led]->BRR = LED_PIN[Led];    
}


/**
 * @brief  Toggles the selected LED.
 * @param  Led: Specifies the Led to be toggled. 
 *   This parameter can be one of following parameters:
 *     @arg LED1
 * @retval None
 */
void iNEMO_Led_Toggle(Led_TypeDef Led)
{
  LED_PORT[Led]->ODR ^= LED_PIN[Led];
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

/**
 * @}
 */
/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
