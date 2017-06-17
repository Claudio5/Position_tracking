/**
  * @file    utils.h 
  * @author  ART Team IMS-Systems Lab
  * @version V2.0.0
  * @date    01 February 2013
  * @brief   Header for utils.c
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
#ifndef __UTILS_H
#define __UTILS_H

#ifdef __cplusplus
 extern "C" {
#endif 
  
#include "stm32f10x.h"
   
   
/**
 * @addtogroup iNEMO_Discovery_System
 * @{
 */

/** @addtogroup Utils
 * @{
 */

/**
 * @brief Test Status enumerative type definition.
 */
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/**
 * @brief Computes the absolute value of its argument.
 */
#define abs(a) ((a)>0?(a):-(a))

/**
 * @brief UID Address
 */
#define U_ID_Base_Register_Address   (0x1FFFF7E8)
/**
 * @brief Pointer to MCU_ID.
 */
#define MCU_ID ((const unsigned char *)(U_ID_Base_Register_Address))


/** 
 * @addtogroup Utils_Functions           Utils Functions
 * @{
 */
void TimerFindFactors(u32 n, uint16_t *a, uint16_t *b);
void Delay(vu32 nCount);
void CopyBuffer(unsigned char* pBufferOrigin, unsigned char* pBufferDestination,  uint8_t NBOfBytes);
void s16_to_u8_buffer(int16_t* refvalue, unsigned char* pBufferDestination);
void u16_to_u8_buffer(uint16_t* refvalue, unsigned char* pBufferDestination);
void s32_to_u8_buffer(int32_t* refvalue, unsigned char* pBufferDestination);
void Fill_Buffer(u32 *pBuffer, uint16_t BufferLenght, u32 Offset);
TestStatus Buffercmp(u32* pBuffer1, u32* pBuffer2, uint16_t BufferLength);
TestStatus eBuffercmp(u32* pBuffer, uint16_t BufferLength);
void Float_To_Buffer(float fInput, uint8_t* ucBuffer);
float Buffer_To_Float(uint8_t* ucBuffer);
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

#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
