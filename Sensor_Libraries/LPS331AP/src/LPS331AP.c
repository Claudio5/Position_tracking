/**
  * @file    LPS331AP.c
  * @author  ART Team IMS-Systems Lab
  * @version V2.3.0
  * @date    12 April 2012
  * @brief   This file provides a set of functions needed to manage the LPS331AP slave.
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


#include "LPS331AP.h"


/**
 * @addtogroup Sensor_Libraries           Sensor Libraries
 * @{
 */


/**
* @addtogroup LPS331AP
* @{
*/


/**
 * @defgroup LPS331AP_Private_TypesDefinitions      LPS331AP Private TypesDefinitions
 * @{
 */


/**
 *@}
 */


/**
 * @defgroup LPS331AP_Private_Defines               LPS331AP Private Defines
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup LPS331AP_Private_Macros               LPS331AP Private Macros
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup LPS331AP_Private_Variables             LPS331AP Private Variables
 * @{
 */


/**
 *@}
 */



/**
 * @defgroup LPS331AP_Private_FunctionPrototypes    LPS331AP Private FunctionPrototypes
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup LPS331AP_Private_Functions             LPS331AP Private Functions
 * @{
 */

/**
* @brief  Set configuration of Pressure Sensor measurement of LPS331AP
* @brief  Set configuration of pressure sensor LPS331AP
* @param  pxLPS331InitStruct : pointer to a LPS331Init structure that contains the configuration setting for the LPS331AP.
* @retval None.
* @details
* <b>Example:</b>
* @code
*  LPS331Init xLps331apInit;
*
*  xLps331apInit.xOutputDataRate=ODR_P_7HZ_T_1HZ;
*  xLps331apInit.xPresRes=LPS_PRESS_AVG_1;
*  xLps331apInit.xTempRes=LPS_TEMP_AVG_1;
*  xLps331apInit.xPressureAutoZero=LPS_DISABLE;
*  xLps331apInit.fPressureRef=0;
*  xLps331apInit.xBDU=LPS_DISABLE;
*  xLps331apInit.xTemperatureSensor = LPS_ENABLE;
*  
*  Lps331apInit(&xLps331apInit);
* @endcode
*/
void Lps331apConfig(LPS331Init* pxLPS331InitStruct)
{
  uint8_t tempReg[3];
  
  for(uint8_t i=0 ; i<3 ; i++)
    tempReg[i]=(uint8_t)(((uint32_t)(4096.0*pxLPS331InitStruct->fPressureRef))>>(8*i));
      
  /* Write the reference value on register */
  Lps331apBufferWrite(tempReg, LPS_REF_P_XL_ADDR,3);
  
  /* Resolution register setting */
  tempReg[0] = (uint8_t)pxLPS331InitStruct->xPresRes | (uint8_t)pxLPS331InitStruct->xTempRes;
    
  /* Write the value on register */ 
  Lps331apByteWrite(&tempReg[0], LPS_RES_CONF_ADDR);
  
  /* Read the CTRL1 and 2 register content */
  Lps331apBufferRead(tempReg, LPS_CTRL_REG1_ADDR,2); 
  
  /* Enter the SDN mode */
  tempReg[0] &= 0x7F;
  Lps331apByteWrite(tempReg, LPS_CTRL_REG1_ADDR);
  
  /* CTRL1 register */
  tempReg[0] &= 0x8B;
  tempReg[0] |= pxLPS331InitStruct->xOutputDataRate;
    
  if(pxLPS331InitStruct->xBDU)
    tempReg[0] |= 0x04;
  else
    tempReg[0] &= 0xFB;
    
  /* CTRL2 register */
  if(pxLPS331InitStruct->xPressureAutoZero)
    tempReg[1] |= 0x02;
  else
    tempReg[1] &= 0xFD;

  /* Write the reference value on register */
  Lps331apBufferWrite(tempReg, LPS_CTRL_REG1_ADDR,2);
  
  /* Exit the SDN mode */
  tempReg[0] |= 0x80;
  Lps331apByteWrite(tempReg, LPS_CTRL_REG1_ADDR);
  
}

/**
* @brief  Gets the general configuration of LPS331AP.
* @param  pxLPS331InitStruct : pointer to a LPS331Init structure that will
*         contain the configuration setting read from the LPS331AP registers.
* @retval None
*/
void Lps331apGetInfo(LPS331Init* pxLPS331InitStruct)
{
  uint8_t tempReg[4];
  
  Lps331apBufferRead(tempReg, LPS_REF_P_XL_ADDR,3);
  tempReg[3]=0;
  
  /* Get the reference value */
  pxLPS331InitStruct->fPressureRef = (float)(*((uint32_t*)tempReg))/4096.0;

  /* Read register */ 
  Lps331apByteRead(&tempReg[0], LPS_RES_CONF_ADDR);
  
  pxLPS331InitStruct->xPresRes=(LPSPressureResolution)(tempReg[0] & 0x0F);
  pxLPS331InitStruct->xTempRes=(LPSTemperatureResolution)(tempReg[0] & 0x70);
  
  /* Read the CTRL1 and 2 register content */
  Lps331apBufferRead(tempReg, LPS_CTRL_REG1_ADDR,2);
  
  /* get info */
  if(tempReg[0] & 0x04)
    pxLPS331InitStruct->xBDU = LPS_ENABLE;
  else
    pxLPS331InitStruct->xBDU = LPS_DISABLE;
   

  if(tempReg[1] & 0x02)
    pxLPS331InitStruct->xPressureAutoZero = LPS_ENABLE;
  else
    pxLPS331InitStruct->xPressureAutoZero = LPS_DISABLE;
    
  /* Get the ODR info */
  Lps331apByteRead(tempReg, LPS_CTRL_REG1_ADDR);
  
  pxLPS331InitStruct->xOutputDataRate = (LPSOutputDataRate)(tempReg[0] & 0x70);
  
}


/**
* @brief  Enable or disable the lowpower mode for pressure sensor LPS331AP.
* @param  xFunctionalState : new state for the lowpower mode.
*         This parameter can be:  LPS_ENABLE or LPS_DISABLE
* @retval None
*/
void Lps331apLowPowerMode(LPSFunctionalState xFunctionalState)
{
  uint8_t tmpreg;
  
  /* Read the register content */
  Lps331apByteRead(&tmpreg, LPS_AMP_CTRL_ADDR);
  
  /* modify the specified bit */
  if(xFunctionalState == LPS_ENABLE)
  {
    tmpreg |= 0x01;
  }
  else
  {
    tmpreg &= 0xFE;
  }
  
  /* Write the computed values on register */
  Lps331apByteWrite(&tmpreg, LPS_AMP_CTRL_ADDR);
}


/**
* @brief  Change pressure resolution for Pressure sensor LPS331AP.
* @param  xPressureResolution : new pressure resolution value. 
*         This parameter can be one of the @ref LPSPressureResolution value.
* @retval None
*/
void Lps331apSetPressureResolution(LPSPressureResolution xPressureResolution)
{
  uint8_t tempReg;
  
  /* Read the register content */
  Lps331apByteRead(&tempReg, LPS_RES_CONF_ADDR);
  
  /* CTRL1 register */
  tempReg &= 0xF0;
  tempReg |= xPressureResolution;
   
  /* Write computed byte onto register */
  Lps331apByteWrite(&tempReg, LPS_RES_CONF_ADDR);
}


/**
* @brief  Change temperature resolution for Pressure sensor LPS331AP.
* @param  xTemperatureResolution : new temperature resolution value. 
*         This parameter can be one of the @ref LPSTemperatureResolution value.
* @retval None
*/
void Lps331apSetTemperatureResolution(LPSTemperatureResolution xTemperatureResolution)
{
  uint8_t tempReg;
  
  /* Read the register content */
  Lps331apByteRead(&tempReg, LPS_RES_CONF_ADDR);
  
  /* CTRL1 register */
  tempReg &= 0x8F;
  tempReg |= xTemperatureResolution;
   
  /* Write computed byte onto register */
  Lps331apByteWrite(&tempReg, LPS_RES_CONF_ADDR);
}


/**
* @brief  Change the ODR(Output data rate) for pressure sensor LPS331AP.
* @param  xDataRate : new ODR value. 
*         This parameter can be one of the @ref LPSOutputDataRate value.
* @retval None
*/
void Lps331apSetDataRate(LPSOutputDataRate xDataRate)
{
  uint8_t tempReg;
  
  /* Read the register content */
  Lps331apByteRead(&tempReg, LPS_CTRL_REG1_ADDR);
  
  /* CTRL1 register */
  tempReg &= 0x8F;
  tempReg |= xDataRate;
  
  /*Enter Power Down mode*/
  Lps331apEnterShutdownCmd();
  
  /* Write computed byte onto register */
  Lps331apByteWrite(&tempReg, LPS_CTRL_REG1_ADDR);
  
  /*Exit from Power down mode*/
  Lps331apExitShutdownCmd();
}


/**
* @brief  Returns the output data rate.
* @retval Datarate in Hz.
*         This parameter is a @ref LPSOutputDataRate.
*/
LPSOutputDataRate Lps331apGetDataRate(void)
{
  uint8_t tmpReg;
  
  /* Read the register content */
  Lps331apByteRead(&tmpReg, LPS_CTRL_REG1_ADDR);
  
  /* ..mask it */
  tmpReg &= 0x70;

  /* return the correspondent value */
  return ((LPSOutputDataRate)tmpReg);
}



/**
* @brief  Reboot memory content of LPS331AP.
* @retval None
*/
void Lps331apRebootCmd(void)
{
  uint8_t tmpReg;
  
  /* Read the register content */
  Lps331apByteRead(&tmpReg, LPS_CTRL_REG2_ADDR);
  
  /* Set the BOOT bit */
  tmpReg |= 0x80;
  
  /* Write register */
  Lps331apByteWrite(&tmpReg, LPS_CTRL_REG2_ADDR);
  
}



/**
* @brief  Enter the shutdown mode for LPS331AP.
* @retval None
*/
void Lps331apEnterShutdownCmd(void)
{
  uint8_t tmpReg;
  
  /* Read the register content */
  Lps331apByteRead(&tmpReg, LPS_CTRL_REG1_ADDR);
  
  /* Reset the power down bit */
  tmpReg &= 0x7F;
  
  /* Write register */
  Lps331apByteWrite(&tmpReg, LPS_CTRL_REG1_ADDR);
  
}


/**
* @brief  Exit the shutdown mode for LPS331AP.
* @retval None
*/
void Lps331apExitShutdownCmd(void)
{
  uint8_t tmpReg;
  
  /* Read the register content */
  Lps331apByteRead(&tmpReg, LPS_CTRL_REG1_ADDR);
  
  /* Set the power down bit */
  tmpReg |= 0x80;
  
  /* Write register */
  Lps331apByteWrite(&tmpReg, LPS_CTRL_REG1_ADDR);
  
}

/**
* @brief Read LPS331AP output register, and calculate the raw  pressure.
* @retval int32_t: pressure raw value.
*/
int32_t Lps331apReadRawPressure(void)
{
  uint8_t buffer[3];
  uint32_t tempVal=0;
  
  /* Read the register content */
  Lps331apBufferRead(buffer, LPS_PRESS_POUT_XL_ADDR, 3);
  
  /* Build the raw data */
  for(uint8_t i=0; i<3; i++)
      tempVal |= (((uint32_t)buffer[i]) << (8*i));
  
  /* convert the 2's complement 24 bit to 2's complement 32 bit */
  if(tempVal & 0x00800000)
    tempVal |= 0xFF000000;
  
  /* return the built value */
  return ((int32_t)tempVal);
    
}


/**
* @brief Read LPS331AP output register, and calculate the raw temperature.
* @retval int16_t: temperature raw value.
*/
int16_t Lps331apReadRawTemperature(void)
{
  uint8_t buffer[2];
  uint16_t tempVal=0;
  
  /* Read the register content */
  Lps331apBufferRead(buffer, LPS_TEMP_OUT_L_ADDR, 2);
  
  /* Build the raw value */
  tempVal = (((uint16_t)buffer[1]) << 8)+(uint16_t)buffer[0];
  
  /* Return it */
  return ((int16_t)tempVal);
    
}


/**
* @brief  Configures the LPS331AP IRQ outout pins.
* @param  pxLPSIrqInit: pointer to the LPSIrqInit structure that defines the IRQ parameters.
* @retval None.
* @details
* <b>Example:</b>
* @code
*  LPSIrqInit xLps331apIrqInit;
*
*  xLps331apIrqInit.xIrqActiveLow = LPS_DISABLE;
*  xLps331apIrqInit.xOutType = LPS_PP;
*  xLps331apIrqInit.xInt1List = LPS_DATA_READY;
*  xLps331apIrqInit.xInt2List = LPS_P_LOW_HIGH;
*  xLps331apIrqInit.fPressThr = 1006.2;
*  xLps331apIrqInit.xIrqPressLow=LPS_ENABLE;
*  xLps331apIrqInit.xIrqPressHigh=LPS_ENABLE;
*
*  ExtiConfiguration();     // set the micro exti before init
*  Lps331apIrqInit(&xLps331apIrqInit);
* @endcode
*/
void Lps331apIrqInit(LPSIrqInit* pxLPSIrqInit)
{
  uint8_t tempReg[2];
  
  /* From the structure build the value to write on CTRL3 reg */
  tempReg[0] = (((uint8_t)(pxLPSIrqInit->xIrqActiveLow))<<7) | ((uint8_t)(pxLPSIrqInit->xOutType)) | ((uint8_t)(pxLPSIrqInit->xInt2List)<<3) | ((uint8_t)(pxLPSIrqInit->xInt1List));

  /* Read the register content */
  Lps331apByteWrite(tempReg, LPS_CTRL_REG3_ADDR);
  
  /* Build the threshold register values */
  tempReg[0]=(uint8_t)(16.0*pxLPSIrqInit->fPressThr);
  tempReg[1]=(uint8_t)(((uint16_t)(16.0*pxLPSIrqInit->fPressThr))>>8);  
  
  /* Read the registers content */
  Lps331apBufferWrite(tempReg, LPS_THS_P_LOW_REG_ADDR, 2);
  
  Lps331apByteRead(tempReg, LPS_INT_CFG_REG_ADDR);
  
  /* Enable or disable the high pressure interrupt */
  if(pxLPSIrqInit->xIrqPressHigh)
    tempReg[0] |= 0x01;
  else
    tempReg[0] &= 0xFE;
  
  /* Enable or disable the low pressure interrupt */
  if(pxLPSIrqInit->xIrqPressLow)
    tempReg[0] |= 0x02;
  else
    tempReg[0] &= 0xFD;
  
  tempReg[0]|=0x04;
  
  /* Write the register or disable the high pressure interrupt */
  Lps331apByteWrite(tempReg, LPS_INT_CFG_REG_ADDR);
  
  /* if one has been requested then enable the differential circuit */
  Lps331apByteRead(tempReg, LPS_CTRL_REG1_ADDR);
  if(pxLPSIrqInit->xIrqPressHigh || pxLPSIrqInit->xIrqPressLow)
  {    
    tempReg[0] |= 0x08; 
  }
  else
  /* else disable it */
  {
    tempReg[0] &= 0xF7; 
  }
  Lps331apByteWrite(tempReg, LPS_CTRL_REG1_ADDR);
  
}



/**
* @brief  Sets the one-shot bit in order to start acquisition when the ONE SHOT mode has been selected by the ODR configuration.
* @retval None.
*/
void Lps331apOneShot(void)
{
  uint8_t tempReg;
  
  /* Read the CTRL2 register */
  Lps331apByteRead(&tempReg, LPS_CTRL_REG2_ADDR);
    
  /* Set the one shot bit */
  tempReg |= 0x01;
  
  /* Write the CTRL2 register */
  Lps331apByteWrite(&tempReg, LPS_CTRL_REG2_ADDR);

}


/**
* @brief  Gets status for LPS331AP data.
* @retval LPS331DataStatus: Data status in a @ref LPS331DataStatus bitfields structure.
*/
LPS331DataStatus Lps331apGetDataStatus(void)
{
  uint8_t tempReg;
  
  /* Read the status register */
  Lps331apByteRead(&tempReg, LPS_STATUS_REG_ADDR);
  
  /* cast and return it */
  return(*(LPS331DataStatus*)&tempReg);
  
}


/** 
* @brief  Sets the pressure threshold.
* @param  fThreshold: Threshold expressed in mbar.
*         This parameter is a float value.
* @retval None.
*/
void Lps331apSetThreshold(float fThreshold)
{
  uint8_t tempReg[2];
  
  /* Build the threshold register values */
  tempReg[0]=(uint8_t)(16.0*fThreshold);
  tempReg[1]=(uint8_t)(((uint16_t)(16.0*fThreshold))>>8); 
  
  /* write the register content */
  Lps331apBufferWrite(tempReg, LPS_THS_P_LOW_REG_ADDR, 2);
  
}


/** 
* @brief  Gets the pressure threshold.
* @retval float: threshold value expressed in mbar.
*/
float Lps331apGetThreshold(void)
{
  uint8_t tempReg[2];
  
  /* read the register content */
  Lps331apBufferRead(tempReg, LPS_THS_P_LOW_REG_ADDR, 2);
  
  /* return the float value */
  return ((float)((((uint16_t)tempReg[1])<<8) + tempReg[0])/16);
  
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

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
