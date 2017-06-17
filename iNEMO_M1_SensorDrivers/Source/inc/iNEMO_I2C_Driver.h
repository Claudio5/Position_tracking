/**
 * @file    iNEMO_I2C_Driver.h
 * @author  ART Team IMS-Systems Lab
 * @version V2.3.0
 * @date    12 April 2011
 * @brief   Header for iNemo_I2C_Driver.c file
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
#ifndef __iNemoI2CDriver_H
#define __iNemoI2CDriver_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @brief Define for using the DMA in I2C1 TX direction.
 */    
#define I2C1_USE_DMA_TX
   
/**
 * @brief Define for using the DMA in I2C1 RX direction.
 */   
#define I2C1_USE_DMA_RX

/**
 * @brief Define for using the DMA in I2C2 TX direction.
 */    
#define I2C2_USE_DMA_TX    

/**
 * @brief Define for using the DMA in I2C2 RX direction.
 */    
#define I2C2_USE_DMA_RX

/**
* @addtogroup iNemo_Sensor_Drivers           iNemo Sensor Drivers
* @{
*/ 
   
/**
 * @addtogroup iNemoI2CDriver
 * @{
 */


/**
 * @defgroup iNemoI2CDriver_Exported_Types      iNemoI2CDriver Exported Types
 * @{
 */
   
/**
 * @}
 */      
  


/** @defgroup iNemoI2CDriver_Exported_Functions         iNemoI2CDriver Exported Functions
 * @{
 */

/**
 * @brief Mapping of the iNemoI2C1Init I2C1 initialization function in the @ref iNemoI2CInit.
 */    
#define iNemoI2C1Init(I2CxSpeed)           iNemoI2CInit(I2C1,I2CxSpeed)
   
/**
 * @brief Mapping of the iNemoI2C1Init I2C2 initialization function in the @ref iNemoI2CInit.
 */     
#define iNemoI2C2Init(I2CxSpeed)           iNemoI2CInit(I2C2,I2CxSpeed)   
   
#ifdef I2C1_USE_DMA_RX
/**
 * @brief Mapping of the iNemoI2C1BufferRead function in the @ref iNemoI2CBufferReadDma in case the DMA for I2C1 RX is enabled.
 */     
#define iNemoI2C1BufferRead(cAddr,pcBuffer,cReadAddr,cNumByteToRead)    iNemoI2CBufferReadDma(I2C1,cAddr,pcBuffer,cReadAddr,cNumByteToRead)
#else
/**
 * @brief Mapping of the iNemoI2C1BufferRead function in the @ref iNemoI2CBufferReadPolling in case the DMA for I2C1 RX is not enabled.
 */   
#define iNemoI2C1BufferRead(cAddr,pcBuffer,cReadAddr,cNumByteToRead)    iNemoI2CBufferReadPolling(I2C1,cAddr,pcBuffer,cReadAddr,cNumByteToRead)
#endif
   
#ifdef I2C2_USE_DMA_RX
/**
 * @brief Mapping of the iNemoI2C2BufferRead function in the @ref iNemoI2CBufferReadDma in case the DMA for I2C2 RX is enabled.
 */    
#define iNemoI2C2BufferRead(cAddr,pcBuffer,cReadAddr,cNumByteToRead)    iNemoI2CBufferReadDma(I2C2,cAddr,pcBuffer,cReadAddr,cNumByteToRead)
#else
/**
 * @brief Mapping of the iNemoI2C2BufferRead function in the @ref iNemoI2CBufferReadPolling in case the DMA for I2C2 RX is not enabled.
 */   
#define iNemoI2C2BufferRead(cAddr,pcBuffer,cReadAddr,cNumByteToRead)    iNemoI2CBufferReadPolling(I2C2,cAddr,pcBuffer,cReadAddr,cNumByteToRead)
#endif
   
#ifdef I2C1_USE_DMA_TX
/**
 * @brief Mapping of the iNemoI2C1BufferWrite function in the @ref iNemoI2CBufferWriteDma in case the DMA for I2C1 TX is enabled.
 */   
#define iNemoI2C1BufferWrite(cAddr,pcBuffer,cWriteAddr,cNumByteToWrite)   iNemoI2CBufferWriteDma(I2C1,cAddr,pcBuffer,cWriteAddr,cNumByteToWrite)   
#else
/**
 * @brief Mapping of the iNemoI2C1BufferWrite function in the @ref iNemoI2CBufferWritePolling in case the DMA for I2C1 TX is not enabled.
 */   
#define iNemoI2C1BufferWrite(cAddr,pcBuffer,cWriteAddr,cNumByteToWrite)   iNemoI2CBufferWritePolling(I2C1,cAddr,pcBuffer,cWriteAddr,cNumByteToWrite)   
#endif
   
#ifdef I2C2_USE_DMA_TX
/**
 * @brief Mapping of the iNemoI2C2BufferWrite function in the @ref iNemoI2CBufferWriteDma in case the DMA for I2C2 TX is enabled.
 */    
#define iNemoI2C2BufferWrite(cAddr,pcBuffer,cWriteAddr,cNumByteToWrite)   iNemoI2CBufferWriteDma(I2C2,cAddr,pcBuffer,cWriteAddr,cNumByteToWrite)   
#else
/**
 * @brief Mapping of the iNemoI2C2BufferWrite function in the @ref iNemoI2CBufferWritePolling in case the DMA for I2C2 TX is not enabled.
 */   
#define iNemoI2C2BufferWrite(cAddr,pcBuffer,cWriteAddr,cNumByteToWrite)   iNemoI2CBufferWritePolling(I2C2,cAddr,pcBuffer,cWriteAddr,cNumByteToWrite)   
#endif
   

void iNemoI2CInit(I2C_TypeDef* I2Cx, uint32_t I2CxSpeed);

void iNemoI2CBufferReadDma(I2C_TypeDef* I2Cx, uint8_t cAddr, uint8_t* pcBuffer, uint8_t cReadAddr, uint8_t cNumByteToRead);
void iNemoI2CBufferReadPolling(I2C_TypeDef* I2Cx, uint8_t cAddr, uint8_t* pcBuffer, uint8_t cReadAddr, uint8_t cNumByteToRead);
void iNemoI2CBufferWriteDma(I2C_TypeDef* I2Cx, uint8_t cAddr, uint8_t* pcBuffer, uint8_t cWriteAddr, uint8_t cNumByteToWrite);
void iNemoI2CBufferWritePolling(I2C_TypeDef* I2Cx, uint8_t cAddr, uint8_t* pcBuffer, uint8_t cWriteAddr, uint8_t cNumByteToWrite);

void iNemoI2CDMAInit(I2C_TypeDef* I2Cx);
void iNemoI2CDMAConfig(I2C_TypeDef* I2Cx, uint8_t* pBuffer, uint32_t lBufferSize, uint32_t Direction);


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

#endif /* __iNemoI2CDriver_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

