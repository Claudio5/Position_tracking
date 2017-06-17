/**
  * @file    iNemoConf.h
  * @author  ART Team IMS-Systems Lab
  * @version V2.3.0
  * @date    01 February 2013
  * @brief   Entry point for iNEMO platform configuration.
  *          Comment the line below to disable the specific sensor and features inclusion 
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

#ifndef __iNEMO_CONF_H
#define __iNEMO_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif 
  
/**
 * @addtogroup iNEMO_Discovery_System
 * @{
 */
   
/**
 * @addtogroup iNEMO_Config                 iNemo Configuration
 * @brief This module is used to enable or disable sensors and other peripherials.
 * @details The MEMS libraries are called in response to the configuration commands from the USB.
 *          The complete flow execution chain for commands is: 
 *            iNemoGuiLayer (frame) -> iNemoConfiguration -> SensorLibraries
 * @{
 */
/**
 * @brief Boolean typedef.
 */  
typedef enum
{
  FALSE = 0, TRUE  = !FALSE
}
bool;

/**
 * @addtogroup iNEMO_Config_System_Defines       System Defines
 * @brief These defines allows to choose the functions to make 
 *         some functions atomic or enter in critical section.
 *         In this case we are using the FreeRTOS API.
 * @{
 */
/**
 * @brief Define related to use of FreeRTOS.
 */   
#define FREERTOS

#ifdef FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif

#ifdef FREERTOS
/**
 * @brief Macro to enter in critical section in case FreeRTOS is used .
 */
#define iNEMO_ENTER_CRITICAL()  taskENTER_CRITICAL()
/**
 * @brief Macro to exit in critical section in case FreeRTOS is used .
 */
#define iNEMO_EXIT_CRITICAL()   taskEXIT_CRITICAL()
#elif defined (USER_SCHEDULER)
/**
 * @brief Macro to enter in critical section in case a user scheduler is used .
 */
#define iNEMO_ENTER_CRITICAL()  userENTER_CRITICAL()
/**
 * @brief Macro to exit in critical section in case a user scheduler is used .
 */
#define iNEMO_EXIT_CRITICAL()   userEXIT_CRITICAL()
#elif defined (NO_SCHEDULER)
/**
 * @brief Macro to enter in critical section under user definition.
 */
#define iNEMO_ENTER_CRITICAL()  
/**
 * @brief Macro to exit in critical section under user definition.
 */
#define iNEMO_EXIT_CRITICAL()   
#endif

/**
 * @}
 */

/** 
 * @addtogroup iNEMO_Config_Sensor_Defines       Sensor Defines
 * @brief Comment the line below to disable the specific sensor and features inclusion.
 * @{
 */

/**
 * @brief AHRS Library Available
 */
#define AHRS_MOD


/**
 * @brief Compass Library Available
 */
#define COMPASS_MOD


/**
 * @brief TRACE Available
 */
#define TRACE_MOD


/**
 * @brief Enable 6-axis module  ACC+MAGNETOMETER 
 */
#define _6X


/**
 * @brief Enable Gyroscope
 */
#define  _GYRO


/**
 * @brief PRESSURE sensor
 */
#define _PRESS


/** 
 * @brief TEMPERATURE sensor
 */
#define _TEMP


/**
 * @brief USER_LED 
 */
#define _ULED

/**
 * @defgroup USER_LED_Constants         USER LED Constants
 * @{
 */
/**
 * @brief Number of LED available 
 */   
#define iNEMO_LEDn                             1

/**
 * @brief This define sets STM32 Pin hooked to LED1.
 */     
#define iNEMO_LED1_PIN                         GPIO_Pin_1

/**
 * @brief This define sets STM32 Port hooked to LED1.
 */   
#define iNEMO_LED1_GPIO_PORT                   GPIOA
   
/**
 * @brief This define sets STM32 clock port hooked to LED1.
 */    
#define iNEMO_LED1_GPIO_CLK                    RCC_APB2Periph_GPIOA


/**
 * @}
 */

/**
 * @brief PUSH_BUTTON 
 * @note The push button is not used in this firmware.
 */
//#define _PBUTT


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
