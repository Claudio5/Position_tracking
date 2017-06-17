/**
  * @file    utils.c 
  * @author  ART Team IMS-Systems Lab
  * @version V2.3.0
  * @date    01 February 2013
  * @brief   This file includes some utility functions
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



#include "utils.h"

/**
 * @addtogroup iNEMO_Discovery_System
 * @{
 */

/**
 * @defgroup Utils      Utils
 * @brief This module contains some useful general purpose functions.
 * @{
 */

/**
 * @addtogroup Utils_Functions
 * @{
 */

/**
 * @brief  Compute two integer value a and b such that n = a * b. It used to
 *                  setup the timer to generate an IRQ whit a specified frequency n.
 * @param  n : the specified frequency
 * @param  a : prescaler factor
 * @param  b : period factor
 * @retval None.
*/
void TimerFindFactors(u32 n, uint16_t *a, uint16_t *b)
{
	/** This function is copied from the ST STR7 library and is
	 * copyright STMicroelectronics.  Reproduced with permission.
	*/

	uint16_t b0;
	uint16_t a0;
	long err, err_min=n;


	*a = a0 = ((n-1)/0xffff) + 1;
	*b = b0 = n / *a;

	for (; *a < 0xffff-1; (*a)++)
	{
		*b = n / *a;
		err = (long)*a * (long)*b - (long)n;
		if (abs(err) > (*a / 2))
		{
			(*b)++;
			err = (long)*a * (long)*b - (long)n;
		}
		if (abs(err) < abs(err_min))
		{
			err_min = err;
			a0 = *a;
			b0 = *b;
			if (err == 0) break;
		}
	}

	*a = a0;
	*b = b0;
}

/**
 * @brief  Inserts a delay time
 * @param  nCount: specifies the delay time length
 * @retval None.
*/

void Delay(vu32 nCount)
{
  for(; nCount != 0; nCount--);
}


/**
 *  @brief Copy one buffer to another
 *  @param pBufferOrigin buffer to copy.
 *  @param pBufferDestination destination buffer.
 *  @param NBOfBytes : number of bytes to copy
 *  @retval None.
 */
void CopyBuffer(unsigned char* pBufferOrigin, unsigned char* pBufferDestination,  uint8_t NBOfBytes)
{
  while(NBOfBytes!=0)
  {
    NBOfBytes--;
    *pBufferDestination=*pBufferOrigin;
    pBufferDestination++;
    pBufferOrigin++;
  }
}


/**
 * @brief Put in a buffer as uint8_t an int16_t.
 * @param refvalue : int16_t recipient
 * @param pBufferDestination : uint8_t buffer destination
 * @retval None
 */

void s16_to_u8_buffer(int16_t* refvalue, unsigned char* pBufferDestination)
{
  uint16_t tmp=0x00;
  tmp = *(uint16_t*)refvalue;
  *pBufferDestination=(uint8_t)(tmp>>8);
  pBufferDestination++;
  *pBufferDestination=(uint8_t)tmp;

}

/**
 * @brief Put in a buffer as uint8_t an uint16_t.
 * @param refvalue : uint16_t recipient
 * @param pBufferDestination : uint8_t buffer destination
 * @retval None
 */
void u16_to_u8_buffer(uint16_t* refvalue, unsigned char* pBufferDestination)
{
  uint16_t tmp=0x00;
  tmp = (uint16_t)(*refvalue);
  *pBufferDestination=(uint8_t)(tmp>>8);
  pBufferDestination++;
  *pBufferDestination=(uint8_t)tmp;
}

/** 
 * @brief Put in a buffer as uint8_t an int32_t.
 * @param refvalue : int32_t recipient
 * @param pBufferDestination : uint8_t buffer destination
 * @retval None
 */
void s32_to_u8_buffer(int32_t* refvalue, unsigned char* pBufferDestination)
{
  uint32_t tmp=0x00;
  tmp = (uint32_t)(*refvalue);
  for (int i=3; i>0; i--)
  {
    *pBufferDestination=(uint8_t)((tmp>>(8*i))&0xff);
    pBufferDestination++;
   }
  *pBufferDestination=(uint8_t)(tmp&0xff);
}

/**
 * @brief Compares two buffers
 * @param pBuffer1 : buffers to be compared
 * @param pBuffer2 : buffers to be compared
 * @param BufferLength : length of buffer 
 * @retval TestStatus :  PASSED if  pBuffer1 is identical to pBuffer2; or FAILED if pBuffer1 id differs from pBuffer2
 */
TestStatus Buffercmp(u32* pBuffer1, u32* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
    {
      return FAILED;
    }

    pBuffer1++;
    pBuffer2++;
  }

  return PASSED;
}


/**
 * @brief Fills buffer with user predefined data
 * @param pBuffer : pointer on the Buffer to fill
 * @param BufferLenght : size of the buffer to fill
 * @param Offset : first value to fill on the Buffer
 * @retval None
 */
void Fill_Buffer(u32 *pBuffer, uint16_t BufferLenght, u32 Offset)
{
  uint16_t index = 0;

  /* Put in global buffer same values */
  for (index = 0; index < BufferLenght; index++ )
  {
    pBuffer[index] = index + Offset;
  }
}


/**
 * @brief Checks if a buffer has all its values are equal to zero
 * @param pBuffer :  buffer to be compared.
 * @param BufferLength : buffer's length
 * @retval TestStatus : PASSED pBuffer values are zero
 *                 		 FAILED At least one value from pBuffer buffer is diffrent
 *                          from zero.
 */
TestStatus eBuffercmp(u32* pBuffer, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer != 0x00)
    {
      return FAILED;
    }

    pBuffer++;
  }

  return PASSED;
}


/**
  * @brief Copy a float, into a 8 bits buffer
  * @param fInput the float to copy into the buffer
  * @param ucBuffer the output buffer
  * @retval None
  */
void Float_To_Buffer(float fInput, uint8_t* ucBuffer)
{
  char* pcData;
  pcData = (char*)(&fInput);
  
  for(int cI=3; cI>=0; cI--)
  {
    ucBuffer[cI]=(char) (*pcData);
    pcData++;
  }
}


/**
  * @brief Cast 4 bytes in float
  * @param ucBuffer the input buffer
  * @retval the data casted in float
  */
float Buffer_To_Float(uint8_t* ucBuffer)
{
  uint32_t nTmp=0;
  int cI;

  for(cI=0 ; cI<4 ; cI++)
    nTmp += ((uint32_t)ucBuffer[cI])<<(8*(3-cI));


  float fData = *(float*)&nTmp;

  return fData;

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
