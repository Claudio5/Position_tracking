/**
  * @file    HAL_LPS331AP.h
  * @author  ART Team IMS-Systems Lab
  * @version V2.3.0
  * @date    12 April 2012
  * @brief   Hardware Abstraction Layer for LPS331AP.
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
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  */


/* Define to prevent recursive inclusion*/
#ifndef __HAL_LPS331AP_H
#define __HAL_LPS331AP_H

/* Includes */
#include "stm32f10x.h"

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @addtogroup iNemo_Sensor_Drivers           iNemo Sensor Drivers
 * @{
 */
   
/**
 * @addtogroup HAL_LPS331AP        HAL LPS331AP
 * @brief This is an adapter header to join the platform indipendent @ref Sensor_Libraries for the LPS331AP pressure sensor
 *        to the low level driver on the microcontroller side.
 * @{
 */

/**
 * @addtogroup  HAL_LPS331AP_Exported_Constants        HAL LPS331AP Exported Constants
 * @{
 */

/**
 * @brief This define is used to define the communication interface for LPS331AP.
 */ 
#define  USE_I2C 
   
/**
 * @brief This define sets the I2C speed of STM32 for the pressure sensor in 400kHz.
 */  
#define LPS_I2C_Speed            400000
   
/**
 * @brief This define sets the Address 0 of LPS331AP.
 */    
#define LPS_I2C_ADDR0            0xBB
   
/**
 * @brief This define sets the Address 1 of LPS331AP.
 */   
#define LPS_I2C_ADDR1            0xB8
   
/**
 *@}
 */


/**
 * @addtogroup  HAL_LPS331AP_Exported_Macros         HAL LPS331AP Exported Macros
 * @{
 */
          
#ifdef USE_I2C
#include "iNEMO_I2C_Driver.h"

/**
 * @brief This define remaps the Lps331apCommInit communication interface initialization in the @ref iNemoI2C1Init.
 */     
#define Lps331apCommInit()                             iNemoI2C1Init(LPS_I2C_Speed)

/**
 * @brief This define remaps the Lps331apBufferRead functions in the @ref iNemoI2C1BufferRead.
 */     
#define Lps331apBufferRead(pVal,cAddress,nBytes)       iNemoI2C1BufferRead(LPS_I2C_ADDR0,pVal,cAddress,nBytes)
   
/**
 * @brief This define remaps the Lps331apBufferWrite functions in the @ref iNemoI2C1BufferWrite.
 */   
#define Lps331apBufferWrite(pVal,cAddress,nBytes)      iNemoI2C1BufferWrite(LPS_I2C_ADDR0,pVal,cAddress,nBytes)

#else
#include "iNEMO_SPI_Driver.h"
   
   // put here the macro definition for spi drivers
   
#endif
   
/**
 *@}
 */

/**
 *@}
 */

/**
 *@}
 */


#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
