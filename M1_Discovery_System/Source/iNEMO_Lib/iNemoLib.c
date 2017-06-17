/**
 * @file    iNemoLib.c
 * @author  ART Team IMS-Systems Lab
 * @version V2.3.0
 * @date    01 February 2013
 * @brief   Sensor Hardware Configuration & Setup & API.
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



/* Includes */
#include "iNemoLib.h"
#include "iNemoConf.h"
#include "utils.h"
#include <string.h>

/**
 * @addtogroup iNEMO_Discovery_System
 * @{
 */

/**
 * @addtogroup iNEMO_Lib              iNemo Library
 * @brief This module is used to configure sensors exporting high level functionalities.
 * @{
 */
/**
 * @brief  Array of iNEMO Sensor parameters.
 */ 
static iNemoSensorPars s_xSensorPars[N_TOT_SAVED_PARS];

#if defined (IAR_ARM_CM3)
/**
 * @brief  Flag stored in Flash at address 0x0807F800 in case of IAR.
 */ 
const uint16_t nErasedFlag @ (FLASH_PARS_ADDR) =0x0300;
#elif defined (KEIL)
/**
 * @brief  Flag stored in Flash at address 0x0807F800 in case of Keil.
 */ 
const uint16_t nErasedFlag __attribute__((at(FLASH_PARS_ADDR))) =0x0300;
#elif defined (CooCox)
/**
 * @brief  Flag stored in Flash at address 0x0807F800 in case of CooCox.
 */ 
__attribute__ ((section(".SECTION251"))) uint16_t nErasedFlag = 0x0300;
#else
#error "No IDE defined"
#endif

/**
 * @brief  Static variable for PC29APR4 version.
 */
static uint8_t s_cVersion=M1_VER_PC29APR4;

/**
 * @brief  Static array configuration for Data Ready Accelerometer IRQ Config.
 */
const static DataReadyIRQConf s_xAccDataReadyConf[2]={
    {
      .lGpioClock=RCC_APB2Periph_GPIOC,
      .pxGpioPort=GPIOC,
      .nGpioPin=GPIO_Pin_8,
      .cGpioPortSource=GPIO_PortSourceGPIOC,
      .cGpioPinSource=GPIO_PinSource8,
      .nExtiLine=EXTI_Line8,
      .nIRQnCh=EXTI9_5_IRQn
    },
    {
      .lGpioClock=RCC_APB2Periph_GPIOB,
      .pxGpioPort=GPIOB,
      .nGpioPin=GPIO_Pin_5,
      .cGpioPortSource=GPIO_PortSourceGPIOB,
      .cGpioPinSource=GPIO_PinSource5,
      .nExtiLine=EXTI_Line5,
      .nIRQnCh=EXTI9_5_IRQn
    }
};

/**
 * @brief  Static array configuration for Data Ready Magnetometer IRQ Config.
 */
const static DataReadyIRQConf s_xMagDataReadyConf[2]={
    {
      .lGpioClock=RCC_APB2Periph_GPIOC,
      .pxGpioPort=GPIOC,
      .nGpioPin=GPIO_Pin_7,
      .cGpioPortSource=GPIO_PortSourceGPIOC,
      .cGpioPinSource=GPIO_PinSource7,
      .nExtiLine=EXTI_Line7,
      .nIRQnCh=EXTI9_5_IRQn
    },
    {
      .lGpioClock=RCC_APB2Periph_GPIOB,
      .pxGpioPort=GPIOB,
      .nGpioPin=GPIO_Pin_4,
      .cGpioPortSource=GPIO_PortSourceGPIOB,
      .cGpioPinSource=GPIO_PinSource4,
      .nExtiLine=EXTI_Line4,
      .nIRQnCh=EXTI4_IRQn
    }
};

/**
 * @addtogroup iNEMO_Config_Macros                   iNEMO Configuration Macros
 * @brief These macros can be used to refer to some sensor parameters in the array inside
 *        an iNemoDataStruct variable in an simple way.
 * @{
 */
/**
 * @brief  Macro for Accelerometer X-Axis sensitivity .
 */ 
#define iNEMO_SensitivityAccX(data)     (data->fSensitivity[0])
/**
 * @brief  Macro for Accelerometer Y-Axis sensitivity .
 */ 
#define iNEMO_SensitivityAccY(data)     (data->fSensitivity[1])
/**
 * @brief  Macro for Accelerometer Z-Axis sensitivity .
 */ 
#define iNEMO_SensitivityAccZ(data)     (data->fSensitivity[2])

/**
 * @brief  Macro for Gyroscope X-Axis sensitivity .
 */ 
#define iNEMO_SensitivityGyroX(data)    (data->fSensitivity[3])
/**
 * @brief  Macro for Gyroscope Y-Axis sensitivity .
 */ 
#define iNEMO_SensitivityGyroY(data)    (data->fSensitivity[4])
/**
 * @brief  Macro for Gyroscope Z-Axis sensitivity .
 */ 
#define iNEMO_SensitivityGyroZ(data)    (data->fSensitivity[5])

/**
 * @brief  Macro for Magnetometer X-Axis sensitivity .
 */ 
#define iNEMO_SensitivityMagnX(data)    (data->fSensitivity[6])
/**
 * @brief  Macro for Magnetometer Y-Axis sensitivity .
 */ 
#define iNEMO_SensitivityMagnY(data)    (data->fSensitivity[7])
/**
 * @brief  Macro for Magnetometer Z-Axis sensitivity .
 */ 
#define iNEMO_SensitivityMagnZ(data)    (data->fSensitivity[8])

/**
 * @brief  Macro for Pressure sensor sensitivity .
 */ 
#define iNEMO_SensitivityPress(data)    (data->fSensitivity[9])
/**
 * @brief  Macro for Temperature sensor sensitivity .
 */ 
#define iNEMO_SensitivityTemp(data)     (data->fSensitivity[10])

/**
 * @brief  Macro for Accelerometer X-Axis offset .
 */ 
#define iNEMO_OffAccX(data)     (data->nOffset[0])
/**
 * @brief  Macro for Accelerometer Y-Axis offset .
 */ 
#define iNEMO_OffAccY(data)     (data->nOffset[1])
/**
 * @brief  Macro for Accelerometer z-Axis offset .
 */ 
#define iNEMO_OffAccZ(data)     (data->nOffset[2])

/**
 * @brief  Macro for Gyroscope X-Axis offset .
 */ 
#define iNEMO_OffGyroX(data)    (data->nOffset[3])
/**
 * @brief  Macro for Gyroscope Y-Axis offset .
 */
#define iNEMO_OffGyroY(data)    (data->nOffset[4])
/**
 * @brief  Macro for Gyroscope Z-Axis offset .
 */
#define iNEMO_OffGyroZ(data)    (data->nOffset[5])

/**
 * @brief  Macro for Magnetometer X-Axis offset .
 */
#define iNEMO_OffMagnX(data)    (data->nOffset[6])
/**
 * @brief  Macro for Magnetometer Y-Axis offset .
 */
#define iNEMO_OffMagnY(data)    (data->nOffset[7])
/**
 * @brief  Macro for Magnetometer Z-Axis offset .
 */
#define iNEMO_OffMagnZ(data)    (data->nOffset[8])

/**
 * @brief  Macro for Pressure sensor offset .
 */
#define iNEMO_OffPress(data)    (data->nOffset[9])
/**
 * @brief  Macro for Temperature sensor offset .
 */
#define iNEMO_OffTemp(data)     (data->nOffset[10])

/**
 * @brief  Macro for Accelerometer X-Axis gain .
 */ 
#define iNEMO_GainAccX(data)     (data->fScaleFactor[0])
/**
 * @brief  Macro for Accelerometer Y-Axis gain .
 */ 
#define iNEMO_GainAccY(data)     (data->fScaleFactor[1])
/**
 * @brief  Macro for Accelerometer Z-Axis gain .
 */ 
#define iNEMO_GainAccZ(data)     (data->fScaleFactor[2])

/**
 * @brief  Macro for Gyroscope X-Axis gain .
 */ 
#define iNEMO_GainGyroX(data)    (data->fScaleFactor[3])
/**
 * @brief  Macro for Gyroscope Y-Axis gain .
 */
#define iNEMO_GainGyroY(data)    (data->fScaleFactor[4])
/**
 * @brief  Macro for Gyroscope Z-Axis gain .
 */
#define iNEMO_GainGyroZ(data)    (data->fScaleFactor[5])

/**
 * @brief  Macro for Magnetometer X-Axis gain .
 */
#define iNEMO_GainMagnX(data)    (data->fScaleFactor[6])
/**
 * @brief  Macro for Magnetometer Y-Axis gain .
 */
#define iNEMO_GainMagnY(data)    (data->fScaleFactor[7])
/**
 * @brief  Macro for Magnetometer Z-Axis gain .
 */
#define iNEMO_GainMagnZ(data)    (data->fScaleFactor[8])

/**
 * @brief  Macro for Pressure sensor gain .
 */
#define iNEMO_GainPress(data)    (data->fScaleFactor[9])
/**
 * @brief  Macro for Temperature sensor gain .
 */
#define iNEMO_GainTemp(data)     (data->fScaleFactor[10])

/**
 * @}
 */


/**
 * @addtogroup iNEMO_Lib_Functions                   iNEMO Library Functions
 * @brief The set of functions used to interface sensors through their libraries and manage the flash in order to permanently store some parameters.
 * @{
 */

/**
 * @brief Get the hardware version for the iNEMO platform.
 * @retval HW Version.
 */
uint8_t iNemoGetVersion(void)
{
  return s_cVersion; 
}

/**
 * @brief Configure the hardware for the iNEMO platform.
 * @retval None.
 */
void iNemoHwConfig(void)
{
#ifdef _ULED
  /* Led */
  iNEMO_Led_Init(LED1);
#endif

#ifdef _PRESS
  /* Communication bus for pressure sensor */
  Lps331apCommInit();
#endif

#ifdef _PBUTT
  /* Push button */
  iNEMO_Button_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
#endif

#ifdef _6X
  /* Communication bus for 6 axis sensor */
  Lsm303dlhcI2CInit();
#endif

#ifdef _GYRO
  /* Communication bus for gyroscope sensor */
  L3gd20CommInit();
#endif
}

#ifdef _6X
/**
 * @brief Configure the DataReady IRQ for the accelerometer of LSM303DLHC.
 * @param xNewState: New state for the data ready configuration for this sensor.
 *                This parameter can be ENABLE or DISABLE.
 * @retval None.
 * @note Be careful in using this function while sharing this EXTI line with other devices
 *        in other GPIO pins.
 */
void iNemoAccDataReadyConfig(FunctionalState xNewState)
{
  RCC_APB2PeriphClockCmd(s_xAccDataReadyConf[s_cVersion].lGpioClock | RCC_APB2Periph_AFIO, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  GPIO_InitStructure.GPIO_Pin = s_xAccDataReadyConf[s_cVersion].nGpioPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(s_xAccDataReadyConf[s_cVersion].pxGpioPort, &GPIO_InitStructure);

  GPIO_EXTILineConfig(s_xAccDataReadyConf[s_cVersion].cGpioPortSource, s_xAccDataReadyConf[s_cVersion].cGpioPinSource);

  EXTI_InitStructure.EXTI_Line = s_xAccDataReadyConf[s_cVersion].nExtiLine;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = xNewState;

  EXTI_Init(&EXTI_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = s_xAccDataReadyConf[s_cVersion].nIRQnCh;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = xNewState;
  NVIC_Init(&NVIC_InitStructure);
  
  EXTI_ClearITPendingBit(s_xAccDataReadyConf[s_cVersion].nExtiLine);
  
  /* Configure the sensor data ready */
  Lsm303dlhcAccIrq1Config(LSM_I1_DRDY1,(LSMFunctionalState)xNewState);
    
}


/**
 * @brief Configure the DataReady IRQ for the magnetometer of LSM303DLHC.
 * @param xNewState: New state for the data ready configuration for this sensor.
 *                This parameter can be ENABLE or DISABLE.
 * @retval None.
 * @note Be careful in using this function while sharing this EXTI line with other devices
 *        in other GPIO pins.
 */
void iNemoMagDataReadyConfig(FunctionalState xNewState)
{
  RCC_APB2PeriphClockCmd(s_xMagDataReadyConf[s_cVersion].lGpioClock | RCC_APB2Periph_AFIO, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  GPIO_InitStructure.GPIO_Pin = s_xMagDataReadyConf[s_cVersion].nGpioPin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(s_xMagDataReadyConf[s_cVersion].pxGpioPort, &GPIO_InitStructure);

  GPIO_EXTILineConfig(s_xMagDataReadyConf[s_cVersion].cGpioPortSource, s_xMagDataReadyConf[s_cVersion].cGpioPinSource);

  EXTI_InitStructure.EXTI_Line = s_xMagDataReadyConf[s_cVersion].nExtiLine;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = xNewState;

  EXTI_Init(&EXTI_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = s_xMagDataReadyConf[s_cVersion].nIRQnCh;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = xNewState;
  NVIC_Init(&NVIC_InitStructure);
  
  EXTI_ClearITPendingBit(s_xMagDataReadyConf[s_cVersion].nExtiLine);
  
  Lsm303dlhcMagDataReadyIrqConfig((LSMFunctionalState)xNewState);
  
}
#endif

#ifdef _GYRO
/**
 * @brief Configure the DataReady IRQ for L3GD20.
 * @param xNewState: New state for the data ready configuration for this sensor.
 *                This parameter can be ENABLE or DISABLE.
 * @retval None.
 * @note Be careful in using this function while sharing this EXTI line with other devices
 *        in other GPIO pins.
 */
void iNemoGyroDataReadyConfig(FunctionalState xNewState)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource6);

  EXTI_InitStructure.EXTI_Line = EXTI_Line6;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = xNewState;

  EXTI_Init(&EXTI_InitStructure);
    
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = xNewState;
  NVIC_Init(&NVIC_InitStructure);  
  
  EXTI_ClearITPendingBit(EXTI_Line6);
  
  /* Configure the sensor data ready */
  L3gd20Irq2Config(L3G_I2_DRDY,(L3GFunctionalState)xNewState);
  
}
#endif

#ifdef _PRESS
/**
 * @brief Configure the DataReady IRQ for LPS331AP.
 * @param xNewState: New state for the data ready configuration for this sensor.
 *                This parameter can be ENABLE or DISABLE.
 * @retval None.
 * @note Be careful in using this function while sharing this EXTI line with other devices
 *        in other GPIO pins.
 */
void iNemoPressDataReadyConfig(FunctionalState xNewState)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  
  LPSIrqList xPressIrqList;
  LPSIrqInit xLps331apIrqInit;
   
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = xNewState;

  EXTI_Init(&EXTI_InitStructure);

  EXTI_ClearITPendingBit(EXTI_Line0);
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = xNewState;
  NVIC_Init(&NVIC_InitStructure);
  
  if(xNewState)
    xPressIrqList=LPS_DATA_READY;
  else
    xPressIrqList=LPS_GND;
  
  /* Fill the sensor IRQ structure */
  xLps331apIrqInit.xIrqActiveLow = LPS_DISABLE;
  xLps331apIrqInit.xOutType = LPS_PP;
  xLps331apIrqInit.xInt1List = xPressIrqList;
  xLps331apIrqInit.xInt2List = LPS_GND;
  xLps331apIrqInit.fPressThr = 0.0;
  xLps331apIrqInit.xIrqPressLow=LPS_DISABLE;
  xLps331apIrqInit.xIrqPressHigh=LPS_DISABLE;
  
  /* Configure the sensor IRQ */
  Lps331apIrqInit(&xLps331apIrqInit);
 
}
#endif


/** 
 * @brief This function initialize the s_xSensorPars struct array.
 *  The structure is used to keep a consistent matching between flash and RAM.
 * @note Every element of this array must be initialized in accord to the parameter size defined by the 
 *   iNemoSetSensor and iNemoGetSensor functions.<br>
 *   ANY MODIFICATION TO THIS STRUCTURE WILL AFFECT THE CORRECT MATCHING BETWEEN FLASH AND RAM DATA FORMAT.
 *   It is recommended not to use this function but @ref iNemoSensorParsStructAutoInit .
 * @retval None.
 */
void iNemoSensorParsStructManInit(void)
{
  uint16_t nOffset=0;
  
  /*Accelerometer*/
  for(uint8_t j=0;j<iNEMO_ACC_N_PARS;j++)
  {
    s_xSensorPars[nOffset+j].cIdSensPar = ((iNEMO_ACC_ID<<4)|j);
    (j<2)?(s_xSensorPars[nOffset+j].cSize = 1) : (s_xSensorPars[nOffset+j].cSize = 2); 
  }
  nOffset += iNEMO_ACC_N_PARS;
  
  /*Mag*/
  for(uint8_t j=0;j<iNEMO_MAG_N_PARS;j++)
  {
    s_xSensorPars[nOffset+j].cIdSensPar = ((iNEMO_MAG_ID<<4)|j);
    (j<3)?(s_xSensorPars[nOffset+j].cSize = 1) : (s_xSensorPars[nOffset+j].cSize = 2); 
  }
  nOffset += iNEMO_MAG_N_PARS;
  
  /*Gyro*/
  for(uint8_t j=0;j<iNEMO_GYRO_N_PARS;j++)
  {
    s_xSensorPars[nOffset+j].cIdSensPar = ((iNEMO_GYRO_ID<<4)|j);
    (j<2)?(s_xSensorPars[nOffset+j].cSize = 1) : (s_xSensorPars[nOffset+j].cSize = 2); 
  }
  nOffset += iNEMO_GYRO_N_PARS;
  
  /*Press*/
  for(uint8_t j=0;j<iNEMO_PRESS_N_PARS;j++)
  {
    s_xSensorPars[nOffset+j].cIdSensPar = ((iNEMO_PRESS_ID<<4)|j);
    (j<1)?(s_xSensorPars[nOffset+j].cSize = 1) : (s_xSensorPars[nOffset+j].cSize = 2); 
  }
  nOffset += iNEMO_PRESS_N_PARS;
  
  /*Temp*/  
  for(uint8_t j=0;j<iNEMO_TEMP_N_PARS;j++)
  {
    s_xSensorPars[nOffset+j].cIdSensPar = ((iNEMO_TEMP_ID<<4)|j);
    s_xSensorPars[nOffset+j].cSize = 2; 
  }
  nOffset += iNEMO_TEMP_N_PARS;
  
}


/** 
 * @brief This function initialize the s_xSensorPars struct array.
 *  The structure is used to keep a consistent matching between flash and RAM.
 * @note Every element of this array is initialized in accord to the parameter size defined by the 
 *   iNemoSetSensor and iNemoGetSensor functions.
 * @retval None.
 */
void iNemoSensorParsStructAutoInit(void)
{
  iNemoData data;
  uint8_t val[10];
  bool isAllowed;
  uint16_t nStructIndex=0;
  
  /* loop over 6 sensors codes */
  for(uint8_t s=0;s<FLASH_METAD_MAX_SENS_ID;s++)
  {
    /* for each of them loop over 10 pars */
    for(uint8_t p=0;p<FLASH_METAD_MAX_PAR_ID;p++)
    {
      /* exploit the @ref iNemoGetSensorParam in order to map the flash image */
      isAllowed=iNemoGetSensorParam(&data, s, p, val);
    
      /* take only the allowed parameters (the for loops use all the numbers between begin and end,
      but only some of them are allowed by @ref iNemoGetSensorParam ) */
      if(isAllowed)
      {
        /* add a new entry in the flash map */
        s_xSensorPars[nStructIndex].cIdSensPar = ((s<<4)|p);
        s_xSensorPars[nStructIndex].cSize = val[0]-2;
        
        nStructIndex++;
      }
    }
  }
  
}


/**
 * @brief Low level function for flash writing.
 * @param lSectStartAddr Start address.
 * @param pcBuffer data buffer to write.
 * @param cSize buffer length.
 * @retval None.
 */
void WriteBufferToFlash(uint32_t lSectStartAddr, uint8_t* pcBuffer, uint8_t cSize)
{
  uint32_t lSectAddr=FLASH_PARS_ADDR;
  
  /* flash unlock */
  FLASH_Unlock();
  
  /* erase flash page starting from the FLASH_PARS_ADDR address */
  FLASH_ErasePage(FLASH_PARS_ADDR);
  
  /* program flash word by word */
  for(uint16_t i=0;i<cSize;i+=4)
  { 
    FLASH_ProgramWord(lSectAddr+i, *(uint32_t *)(pcBuffer + i));
  }
  
  /* flash lock */
  FLASH_Lock();
  
}


/**
 * @brief  Load from flash routine.
 *         This function uses the s_xSensorPars struct array in order to mantain the RAM-Flash matching.
 * @param  pData: pointer to iNemoData structure.
 * @retval None.
 */
void iNemoReadFromFlash(iNemoData* pData)
{
  uint8_t cSensCode,cParCode;
  uint32_t lAddrToRead=FLASH_PARS_ADDR+2;
  

  /* get all the parameters from the data structure */  
  for(uint16_t i=0;i<N_TOT_SAVED_PARS;i++)
  {
    /* read metadata */
    s_xSensorPars[i].cIdSensPar = (*(uint8_t*)lAddrToRead);
    s_xSensorPars[i].cSize = (*(uint8_t*)(lAddrToRead+1));
   
    /* prepare the arguments to pass to iNemoSetSensorParam() */
    cSensCode = (s_xSensorPars[i].cIdSensPar>>4) & 0x0F; 
    cParCode = s_xSensorPars[i].cIdSensPar & 0x0F; 
    
    iNemoSetSensor(pData, cSensCode, cParCode, s_xSensorPars[i].cSize+2, (uint8_t*)(lAddrToRead+2));

    /* increment flash address to read from */
    lAddrToRead += s_xSensorPars[i].cSize+2;
  }
  
}


/**
 * @brief  Save to flash routine.
 * @param  pData: pointer to iNemoData structure.
 * @param  xKeepFlashMap: this parameter is used to discriminate between two cases.
 *         When it is SET then the flash will be written according to the metadata it contains.
 *         When it is RESET the the metadata on flash will be overwritten in accord with the @ref iNemoGetSensorParam
 *         function.
 *   This function uses the s_xSensorPars struct array in order to mantain the RAM-Flash matching.
 * 
 * @retval None.
 */
void iNemoSaveToFlash(iNemoData* pData, FlagStatus xKeepFlashMap)
{
  uint8_t cSensCode,cParCode;
  uint8_t nBytesToSave=2;
  uint8_t pcPars[150];
  uint32_t lAddrToRead=FLASH_PARS_ADDR;
 
  /* Set flash header */
  pcPars[0] = FLASH_DATA_VERSION;
  pcPars[1] = FLASH_STATE_READY;
     
  /* get all the parameters from the data structure */
  for(uint16_t i=0;i<N_TOT_SAVED_PARS;i++)
  {
    if(xKeepFlashMap)
    {
      /* read metadata from flash */
      s_xSensorPars[i].cIdSensPar = (*(uint8_t*)(lAddrToRead+nBytesToSave));
      s_xSensorPars[i].cSize = (*(uint8_t*)(lAddrToRead+nBytesToSave+1));
    }
    else
    {
      /* rebuild metadata into flash */
      iNemoSensorParsStructAutoInit();
    }
    
    /* prepare the arguments to pass to iNemoGetSensorParam() */
    cSensCode = (s_xSensorPars[i].cIdSensPar>>4) & 0x0F; 
    cParCode = s_xSensorPars[i].cIdSensPar & 0x0F; 
    
    iNemoGetSensorParam(pData, cSensCode, cParCode, &pcPars[nBytesToSave]);
       
    pcPars[nBytesToSave] = s_xSensorPars[i].cIdSensPar;
    pcPars[nBytesToSave+1] = s_xSensorPars[i].cSize;
    
    /* build the buffer to save */
    for(uint8_t j=0;j<s_xSensorPars[i].cSize;j++)
    {
       pcPars[nBytesToSave+j+2]=pcPars[nBytesToSave+j+3];
    }
    
    /* increment the bytes to save var */
    nBytesToSave += s_xSensorPars[i].cSize+2;
  }
  
  /* write the buffer on flash */
  WriteBufferToFlash(FLASH_PARS_ADDR,pcPars,nBytesToSave);
    
}


/**
 * @brief  Initialize the iNEMO data structure.
 * @param  pData: pointer to iNemoData structure.
 * @retval None.
 */
void iNemoSensorParametersInit(iNemoData* pData)
{  
  
  if(!(((*(uint8_t*)FLASH_PARS_ADDR) == FLASH_DATA_VERSION) && ((*(uint8_t*)(FLASH_PARS_ADDR+1)) & 0x03) == FLASH_STATE_READY))
  {
    if(nErasedFlag)
      iNEMO_OffAccX(pData)=0;
     
    iNEMO_OffAccX(pData)=0;
    iNEMO_OffAccY(pData)=0;
    iNEMO_OffAccZ(pData)=0;
    
    iNEMO_OffGyroX(pData)=0;
    iNEMO_OffGyroY(pData)=0;
    iNEMO_OffGyroZ(pData)=0;
    
    iNEMO_OffMagnX(pData)=0;
    iNEMO_OffMagnY(pData)=0;
    iNEMO_OffMagnZ(pData)=0;
    
    iNEMO_OffPress(pData)=0;
    iNEMO_OffTemp(pData)=0;
    
    iNEMO_GainAccX(pData)=1;
    iNEMO_GainAccY(pData)=1;
    iNEMO_GainAccZ(pData)=1;
    
    iNEMO_GainGyroX(pData)=1;
    iNEMO_GainGyroY(pData)=1;
    iNEMO_GainGyroZ(pData)=1;
    
    iNEMO_GainMagnX(pData)=1;
    iNEMO_GainMagnY(pData)=1;
    iNEMO_GainMagnZ(pData)=1;
    
    iNEMO_GainPress(pData)=1;
    iNEMO_GainTemp(pData)=1;

    iNemoSensorParsStructAutoInit();
    
    iNemoSaveToFlash(pData,RESET);
    
  }
  else
  {
    if(((*(uint8_t*)(FLASH_PARS_ADDR+1)) & 0x03) == FLASH_STATE_READY)
    {
      iNemoReadFromFlash(pData);
    }
  }

  /* Sensitivities are not saved in flash because derived from the fullscale values by the previously set sensors libraries */
#ifdef _6X
  Lsm303dlhcAccGetSensitivity(&iNEMO_SensitivityAccX(pData));
  Lsm303dlhcMagGetSensitivity(&iNEMO_SensitivityMagnX(pData));
#else
  
  iNEMO_SensitivityAccX(pData)=1;
  iNEMO_SensitivityAccY(pData)=1;
  iNEMO_SensitivityAccZ(pData)=1;
  
  iNEMO_SensitivityMagnX(pData)=1;
  iNEMO_SensitivityMagnY(pData)=1;
  iNEMO_SensitivityMagnZ(pData)=1;
  
#endif
  
#ifdef _GYRO  
  L3gd20GetSensitivity(&iNEMO_SensitivityGyroX(pData));
#else
  iNEMO_SensitivityGyroX(pData)=1;
  iNEMO_SensitivityGyroY(pData)=1;
  iNEMO_SensitivityGyroZ(pData)=1;
#endif
  
  iNEMO_SensitivityPress(pData) = 4096.0;
  iNEMO_SensitivityTemp(pData) = 8.0;
    
}


/**
 * @brief  Initialize the iNEMO sensor platform.
 * @retval None.
 */
void iNemoSensorsConfig(void)
{
  
#ifdef _GYRO
  /* Gyroscope initialization */
  L3GInit L3GInitStructure;
  L3GInitStructure.xPowerMode = L3G_NORMAL_SLEEP_MODE;
  L3GInitStructure.xOutputDataRate = L3G_ODR_95_HZ_CUTOFF_12_5;
  L3GInitStructure.xEnabledAxes = L3G_ALL_AXES_EN;
  L3GInitStructure.xFullScale = L3G_FS_500_DPS;
  L3GInitStructure.xDataUpdate = L3G_BLOCK_UPDATE;
  L3GInitStructure.xEndianness = L3G_BIG_ENDIAN;
  L3gd20Config(&L3GInitStructure);

#endif

#ifdef _6X
  /* Accelerometer initialization */
  LSMAccInit  LSMAccInitStructure;
  LSMAccFilterInit LSMAccFilterInitStructure;

  LSMAccInitStructure.xPowerMode = LSM_NORMAL_MODE;
  LSMAccInitStructure.xOutputDataRate = LSM_ODR_50_HZ;
  LSMAccInitStructure.xEnabledAxes= LSM_ALL_AXES_EN;
  LSMAccInitStructure.xFullScale = LSM_FS_2G;
  LSMAccInitStructure.xDataUpdate = LSM_CONTINUOS_UPDATE;
  LSMAccInitStructure.xEndianness=LSM_BIG_ENDIAN;
  LSMAccInitStructure.xHighResolution=LSM_ENABLE;

  /* Accelerometer high pass filter */
  LSMAccFilterInitStructure.xHPF=LSM_DISABLE;
  LSMAccFilterInitStructure.xHPF_Mode=LSM_HPFM_NORMAL;
  LSMAccFilterInitStructure.cHPFReference=0x00;
  LSMAccFilterInitStructure.xHPFCutOff=LSM_HPCF_16;
  LSMAccFilterInitStructure.xHPFClick=LSM_DISABLE;
  LSMAccFilterInitStructure.xHPFAOI2=LSM_DISABLE;
  LSMAccFilterInitStructure.xHPFAOI1=LSM_DISABLE;

  Lsm303dlhcAccConfig(&LSMAccInitStructure);
  Lsm303dlhcAccFilterConfig(&LSMAccFilterInitStructure);

  /* Magnetometer initialization */
  LSMMagInit LSMMagInitStructure;
  LSMMagInitStructure.xOutputDataRate = LSM_ODR_75_HZ;
  LSMMagInitStructure.xFullScale = LSM_FS_1_3_GA;
  LSMMagInitStructure.xWorkingMode = LSM_CONTINUOS_CONVERSION;
#ifdef LSM_TEMP_ON
  LSMMagInitStructure.xTemperatureSensor = LSM_ENABLE;
#else
  LSMMagInitStructure.xTemperatureSensor = LSM_DISABLE;
#endif
  
  Lsm303dlhcMagConfig(&LSMMagInitStructure);

#endif

#ifdef _PRESS
  /* Pressure sensor initialization */
  LPS331Init xLps331apInit;

  xLps331apInit.xOutputDataRate=ODR_P_25HZ_T_25HZ;
  xLps331apInit.xPresRes=LPS_PRESS_AVG_512;
  xLps331apInit.xTempRes=LPS_TEMP_AVG_64;
  xLps331apInit.xPressureAutoZero=LPS_DISABLE;
  xLps331apInit.fPressureRef=0;
  xLps331apInit.xBDU=LPS_ENABLE;

  Lps331apConfig(&xLps331apInit);
  
#endif

}

/**
 * @brief Detects the M1 version (PC29APR4 or PC29APR1) in order to configure the correct IRQ pins.
 * @retval  None.
 */
void iNemoBoardRecognition(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  uint32_t lNClkCycles;
  uint16_t nPeriod,nPrescaler;
  
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  /* Enable timer clocks */
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );

  
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );
  /* Time base configuration for timer 2 - set the frequency to 100Hz */
  lNClkCycles = (uint32_t)(configCPU_CLOCK_HZ/100);
  /* compute the counter and prescaler */
  TimerFindFactors(lNClkCycles, &nPrescaler, &nPeriod);
  TIM_TimeBaseStructure.TIM_Period = nPeriod - 1;
  TIM_TimeBaseStructure.TIM_Prescaler = nPrescaler - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure );
  TIM_ARRPreloadConfig( TIM2, ENABLE );
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
    
  Lsm303dlhcAccSetDataRate(LSM_ODR_400_HZ);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource8);

  EXTI_InitStructure.EXTI_Line = EXTI_Line8;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  EXTI_ClearITPendingBit(EXTI_Line8);

  /* Configure the sensor data ready */
  Lsm303dlhcAccIrq1Config(LSM_I1_DRDY1,LSM_ENABLE);
  TIM_Cmd( TIM2, ENABLE );
  
  while(!EXTI_GetITStatus(EXTI_Line8) && !TIM_GetITStatus(TIM2, TIM_IT_Update));
  
  TIM_Cmd( TIM2, DISABLE );
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);
  Lsm303dlhcAccIrq1Config(LSM_I1_DRDY1,LSM_DISABLE);
   
  if(EXTI_GetITStatus(EXTI_Line8))
  {
    s_cVersion=M1_VER_PC29APR4;
    
  }
  else
  {
    s_cVersion=M1_VER_PC29APR1;
    
  }
  
  Lsm303dlhcAccSetDataRate(LSM_ODR_50_HZ);
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_Init(&EXTI_InitStructure);
  EXTI_ClearITPendingBit(EXTI_Line8);
}

/**
 * @brief  Change a a configuration parameter of a iNEMO sensor
 * @param pdata : pointer to iNemoData structure.
 * @param cSensor the sensor type. This parameter must be one of @ref Sensor_Management ID.
 * @param cParameter paramenter to change.
 * @param cLength Length of value payload
 * @param pcValue new value to set.
 * @retval boolean This return value says if the operation is allowed or not.
 */
bool iNemoSetSensor(iNemoData* pdata, uint8_t cSensor, uint8_t cParameter, uint8_t cLength, uint8_t* pcValue)
{
  bool isAllowed=FALSE;
      
  for(uint16_t i=0;i<N_TOT_SAVED_PARS;i++)
  {
    if(s_xSensorPars[i].cIdSensPar == ((cSensor<<4)|cParameter)&& s_xSensorPars[i].cSize+2 ==cLength)
    {
      switch(cSensor)
      {
      case 0x00 :  /* Accelerometer */
        isAllowed = iNemoAccSetConfig(pdata, cParameter, pcValue);
        break;
        
      case 0x01 :  /* Magnetometer */
        isAllowed = iNemoMagSetConfig(pdata,cParameter, pcValue);
        break;
        
      case 0x02  :  /* Gyro */
        isAllowed = iNemoGyroSetConfig(pdata,cParameter, pcValue);
        break;
        
      case 0x04  :  /* Pressure Sensor */
        isAllowed = iNemoPressureSetConfig(pdata,cParameter, pcValue);
        break;
        
      case 0x05  : /* Temperature Sensor */
        isAllowed = iNemoTempSetConfig(pdata,cParameter, pcValue);
        break;
      }
    }
    
  }

   return  isAllowed;
}


/**
 * @brief Change a a configuration parameter of a iNEMO sensor
 * @param pdata : poiter to iNemoData structure
 * @param cParameter : paramenter to change  <ul>
 * <li>0x00 - Output data rate</li>
 * <li>0x01 - Full Scale </li>
 * <li>0x02 - HPF  </li>
 * <li>0x03 - Offset X </li>
 * <li>0x04 - Offset Y </li>
 * <li>0x05 - Offset Z </li>
 * <li>0x06 - Scale Factor X </li>
 * <li>0x07 - Scale Factor Y </li>
 * <li>0x08 - Scale Factor Z </li>
 * <li>0xFF - Sensor Name </li>
 * </ul>
 * @param pcValue pointer to the new value to set.
 * @retval boolean This return value says if the operation is allowed or not.
 */
bool iNemoAccSetConfig(iNemoData* pdata, uint8_t cParameter, uint8_t* pcValue)
{
  bool isAllowed=FALSE;

#ifdef _6X
  
  switch(cParameter)
  {
  case 0x00: /* set ODR */

    if(pcValue[0] < 0x0A)
    {

      AccOutputDataRate xOdr,xOdrVerify;
      LSMFunctionalState xLowPower=LSM_DISABLE;
      
      switch(pcValue[0])
      {
      case 0x00: /* 1 Hz */
        xOdr = LSM_ODR_1_HZ;
        break;
      case 0x01: /* 10 Hz */
        xOdr = LSM_ODR_10_HZ;
        break;
      case 0x02: /* 25 Hz */
        xOdr = LSM_ODR_25_HZ;
        break;
      case 0x03: /* 50 Hz */
        xOdr = LSM_ODR_50_HZ;
        break;
      case 0x04: /* 100 Hz */
        xOdr = LSM_ODR_100_HZ;
        break;  
      case 0x05: /* 200 Hz */
        xOdr = LSM_ODR_200_HZ;
        break;
      case 0x06: /* 400 Hz */
        xOdr = LSM_ODR_400_HZ;
        break;
      case 0x07: /* 1620 Hz */
        xOdr = LSM_ODR_1620_HZ;
        xLowPower=LSM_ENABLE;
        break; 
      case 0x08: /* 1344 Hz */
        xOdr = LSM_ODR_1344_HZ;
        break; 
      case 0x09: /* 5376 Hz - adjust for low power (?) */ 
        xOdr = LSM_ODR_1344_HZ;
        xLowPower=LSM_ENABLE;
        break;
      }
      
      Lsm303dlhcAccLowPowerMode(xLowPower);
      Lsm303dlhcAccSetDataRate(xOdr);

      xOdrVerify = Lsm303dlhcAccGetEnumDataRate();

      /* check if the command has been executed */
      if(xOdr == xOdrVerify)
        /* if yes  send ack  */
        isAllowed=TRUE;
      else
        /* else send nack  */
        isAllowed=FALSE;

    }
    else
    {
      isAllowed=FALSE;
    }
    break;

  case  0x01:  /* Set Full scale*/

    if(pcValue[0] < 4 /*&&  pcValue[0] != 0x02*/)
    {

      AccFullScale xFullScale,xFullScaleVerify;

      switch(pcValue[0]){
      case 0:
        xFullScale = LSM_FS_2G;
        iNEMO_SensitivityAccX(pdata)= LSM_Acc_Sensitivity_2g;
        iNEMO_SensitivityAccY(pdata)= LSM_Acc_Sensitivity_2g;
        iNEMO_SensitivityAccZ(pdata)= LSM_Acc_Sensitivity_2g;
        break;
      case 1:
        xFullScale = LSM_FS_4G;
        iNEMO_SensitivityAccX(pdata)= LSM_Acc_Sensitivity_4g;
        iNEMO_SensitivityAccY(pdata)= LSM_Acc_Sensitivity_4g;
        iNEMO_SensitivityAccZ(pdata)= LSM_Acc_Sensitivity_4g;
        break;
      case 2:
        xFullScale = LSM_FS_8G;
        iNEMO_SensitivityAccX(pdata)= LSM_Acc_Sensitivity_8g;
        iNEMO_SensitivityAccY(pdata)= LSM_Acc_Sensitivity_8g;
        iNEMO_SensitivityAccZ(pdata)= LSM_Acc_Sensitivity_8g;
        break;
      case 3:
        xFullScale = LSM_FS_16G;
        iNEMO_SensitivityAccX(pdata)= LSM_Acc_Sensitivity_16g;
        iNEMO_SensitivityAccY(pdata)= LSM_Acc_Sensitivity_16g;
        iNEMO_SensitivityAccZ(pdata)= LSM_Acc_Sensitivity_16g;
        break;
      }

      Lsm303dlhcAccSetFullScale(xFullScale);

      xFullScaleVerify = Lsm303dlhcAccGetEnumFullScale();

      /* check if the command has been executed */
      if(xFullScale == xFullScaleVerify)
        /* if yes  send ack  */
        isAllowed=TRUE;
      else
        /* else send nack  */
        isAllowed=FALSE;

      break;

    case  0x02:  /* HPF */
      {
        LSMAccFilterInit xHighPassFilter;
        xHighPassFilter.xHPFClick = LSM_DISABLE;
        xHighPassFilter.xHPFAOI2 = LSM_DISABLE;
        xHighPassFilter.xHPFAOI1 = LSM_DISABLE;

        uint8_t temp=pcValue[0];
        /*
        pcValue[0] = | RFU |RFU |Filter Enable/Disable |REF ENABLE/DISABLE |HP1 |HP0 | ODR1(not used) |ODR0(not used)
        */
        if((temp & 0x20) == 0x20)  /* check filter ENABLE command */
        {

          xHighPassFilter.xHPF=LSM_ENABLE;
          xHighPassFilter.xHPFCutOff = (AccHPFCutOff)((pcValue[0] & 0x0C)<<2);

          if((temp & 0x10) == 0x10)  /* check if reference is enabled */
          {
            xHighPassFilter.xHPF_Mode = LSM_HPFM_REFERENCE;
            xHighPassFilter.cHPFReference = pcValue[1];

            isAllowed=TRUE;
          }
          else /* reference is disabled */
          {
            xHighPassFilter.xHPF_Mode = LSM_HPFM_NORMAL;
            xHighPassFilter.cHPFReference = 0;

            isAllowed=TRUE;
          }

        }
        else /* filter disable*/
        {
          xHighPassFilter.xHPF = LSM_DISABLE;
          xHighPassFilter.xHPF_Mode = LSM_HPFM_NORMAL;
          xHighPassFilter.cHPFReference = 0;
          xHighPassFilter.xHPFCutOff = LSM_HPCF_8;
          
          isAllowed=TRUE;
        }

        Lsm303dlhcAccFilterConfig(&xHighPassFilter);

      }
      break;

    case 0x03: /* set x axis offset*/ 
      iNEMO_OffAccX(pdata)=((int16_t)pcValue[0] << 8) + pcValue[1];
      isAllowed=TRUE;
      
      break;

    case 0x04: /* set y axis offset*/
     
      iNEMO_OffAccY(pdata)=((int16_t)pcValue[0] << 8) + pcValue[1];
      isAllowed=TRUE;

      break;

    case 0x05: /* set z axis offset*/
     
        iNEMO_OffAccZ(pdata)=((int16_t)pcValue[0] << 8) + pcValue[1];
        isAllowed=TRUE;

      break;

    case 0x06: /* set x axis scale factor */
        iNEMO_GainAccX(pdata)=(float)(((uint16_t)pcValue[0] << 8) + pcValue[1])/1000;
        isAllowed=TRUE;
     
      break;
      
    case 0x07: /* set y axis scale factor */
        iNEMO_GainAccY(pdata)=(float)(((uint16_t)pcValue[0] << 8) + pcValue[1])/1000;
        isAllowed=TRUE;

      break;
    
    case 0x08: /* set z axis scale factor */
        iNEMO_GainAccZ(pdata)=(float)(((uint16_t)pcValue[0] << 8) + pcValue[1])/1000;
        isAllowed=TRUE;
    
      break;
    }

  }

#endif
  
  return isAllowed;

}


/**
 * @brief  Change a  configuration parameter of a iNEMO magnetometer
 * @param pdata : pointer to iNemoData structure
 * @param cParameter : paramenter to change  <ul>
 * <li>0x00 - Output data rate</li>
 * <li>0x01 - Full Scale </li>
 * <li>0x02 - MAGN operationg mode </li>
 * <li>0x03 - Offset X </li>
 * <li>0x04 - Offset Y </li>
 * <li>0x05 - Offset Z </li>
 * <li>0x06 - Scale Factor X </li>
 * <li>0x07 - Scale Factor Y </li>
 * <li>0x08 - Scale Factor Z </li>
 * </ul>
 * @param pcValue pointer to the new value to set.
 * @retval boolean This return value says if the operation is allowed or not.
 */
bool iNemoMagSetConfig( iNemoData * pdata, uint8_t cParameter, uint8_t* pcValue)
{
    bool isAllowed=FALSE;

#ifdef _6X
    
    switch(cParameter)
    {
    case 0x00: /* set ODR */

      if(pcValue[0] < 8)
      {
        MagOutputDataRate xOdr,xOdrVerify;

        switch(pcValue[0]){
        case 0:
          xOdr = LSM_ODR_0_75_HZ;
          break;
        case 1:
          xOdr = LSM_ODR_1_5_HZ;
          break;
        case 2:
          xOdr = LSM_ODR_3_0_HZ;
          break;
        case 3:
          xOdr = LSM_ODR_7_5_HZ;
          break;
        case 4:
          xOdr = LSM_ODR_15_HZ;
          break;
        case 5:
          xOdr = LSM_ODR_30_HZ;
          break;
        case 6:
          xOdr = LSM_ODR_75_HZ;
          break;
        case 7:
          xOdr = LSM_ODR_220_HZ;
          break;

        }

        Lsm303dlhcMagSetDataRate(xOdr);

        /* check if the command has been executed */
        xOdrVerify = Lsm303dlhcMagGetEnumDataRate();

        if(xOdr == xOdrVerify)
          /* if yes  send ack  */
          isAllowed=TRUE;
        else
          /* else send nack  */
          isAllowed=FALSE;

      }
      else
      {
        isAllowed=FALSE;
      }
      break;

    case  0x01:  /* Set Full scale*/


      if(pcValue[0] < 0x08 &&  pcValue[0] > 0x00)
      {
        MagFullScale xFullScale;

        switch(pcValue[0]){
        case 1:
          xFullScale = LSM_FS_1_3_GA;
          iNEMO_SensitivityMagnX(pdata)= LSM_Magn_Sensitivity_XY_1_3Ga;
          iNEMO_SensitivityMagnY(pdata)= LSM_Magn_Sensitivity_XY_1_3Ga;
          iNEMO_SensitivityMagnZ(pdata)= LSM_Magn_Sensitivity_Z_1_3Ga;
          break;
        case 2:
          xFullScale = LSM_FS_1_9_GA;
          iNEMO_SensitivityMagnX(pdata)= LSM_Magn_Sensitivity_XY_1_9Ga;
          iNEMO_SensitivityMagnY(pdata)= LSM_Magn_Sensitivity_XY_1_9Ga;
          iNEMO_SensitivityMagnZ(pdata)= LSM_Magn_Sensitivity_Z_1_9Ga;
          break;
        case 3:
          xFullScale = LSM_FS_2_5_GA;
          iNEMO_SensitivityMagnX(pdata)= LSM_Magn_Sensitivity_XY_2_5Ga;
          iNEMO_SensitivityMagnY(pdata)= LSM_Magn_Sensitivity_XY_2_5Ga;
          iNEMO_SensitivityMagnZ(pdata)= LSM_Magn_Sensitivity_Z_2_5Ga;
          break;
        case 4:
          xFullScale = LSM_FS_4_0_GA;
          iNEMO_SensitivityMagnX(pdata)= LSM_Magn_Sensitivity_XY_4Ga;
          iNEMO_SensitivityMagnY(pdata)= LSM_Magn_Sensitivity_XY_4Ga;
          iNEMO_SensitivityMagnZ(pdata)= LSM_Magn_Sensitivity_Z_4Ga;
          break;
        case 5:
          xFullScale = LSM_FS_4_7_GA;
          iNEMO_SensitivityMagnX(pdata)= LSM_Magn_Sensitivity_XY_4_7Ga;
          iNEMO_SensitivityMagnY(pdata)= LSM_Magn_Sensitivity_XY_4_7Ga;
          iNEMO_SensitivityMagnZ(pdata)= LSM_Magn_Sensitivity_Z_4_7Ga;
          break;

        case 6:
          xFullScale = LSM_FS_5_6_GA;
          iNEMO_SensitivityMagnX(pdata)= LSM_Magn_Sensitivity_XY_5_6Ga;
          iNEMO_SensitivityMagnY(pdata)= LSM_Magn_Sensitivity_XY_5_6Ga;
          iNEMO_SensitivityMagnZ(pdata)= LSM_Magn_Sensitivity_Z_5_6Ga;
          break;

        case 7:
          xFullScale = LSM_FS_8_1_GA;
          iNEMO_SensitivityMagnX(pdata)= LSM_Magn_Sensitivity_XY_8_1Ga;
          iNEMO_SensitivityMagnY(pdata)= LSM_Magn_Sensitivity_XY_8_1Ga;
          iNEMO_SensitivityMagnZ(pdata)= LSM_Magn_Sensitivity_Z_8_1Ga;
          break;
        }

        Lsm303dlhcMagSetFullScale(xFullScale);

        isAllowed=TRUE;

      }
      else
        isAllowed=FALSE;

      break;

    case  0x02:  /* Magn operating mode*/


      if(pcValue[0] < 0x03)
      {
        isAllowed=TRUE;
      }
      else
      {
        isAllowed=FALSE;
      }
      break;

    case 0x03: /* set x axis offset*/
      iNEMO_OffMagnX(pdata)=((int16_t)pcValue[0] << 8) + pcValue[1];
      isAllowed=TRUE;

      break;

    case 0x04: /* set y axis offset*/
      
      iNEMO_OffMagnY(pdata)=((int16_t)pcValue[0] << 8) + pcValue[1];
      isAllowed=TRUE;
     

      break;

    case 0x05: /* set z axis offset*/    
      iNEMO_OffMagnZ(pdata)=((int16_t)pcValue[0] << 8) + pcValue[1];
      isAllowed=TRUE;

      break;
    
    case 0x06: /* set x axis scale factor */
      iNEMO_GainMagnX(pdata)=(float)(((uint16_t)pcValue[0] << 8) + pcValue[1])/1000;
      isAllowed=TRUE;
      
      break;
      
    case 0x07: /* set y axis scale factor */
      iNEMO_GainMagnY(pdata)=(float)(((uint16_t)pcValue[0] << 8) + pcValue[1])/1000;
      isAllowed=TRUE;
      
      break;
    
    case 0x08: /* set z axis scale factor */
      iNEMO_GainMagnZ(pdata)=(float)(((uint16_t)pcValue[0] << 8) + pcValue[1])/1000;
      isAllowed=TRUE;
      
      break;

  }
  
#endif
  
  return isAllowed;

}



/**
 * @brief Change a  configuration parameter of a iNEMO gyro roll-pitch
 * @param pData : pointer to iNemoData structure
 * @param cParameter : paramenter to change  <ul>
 * <li>0x00 - Output Datarate </li>
 * <li>0x01 - Full Scale </li>
 * <li>0x03 - Offset X </li>
 * <li>0x04 - Offset Y </li>
 * <li>0x05 - Offset Z </li>
 * <li>0x06 - Scale Factor X </li>
 * <li>0x07 - Scale Factor Y </li>
 * <li>0x08 - Scale Factor Z </li>
 * </ul>
 * @param pcValue pointer to the new value to set.
 * @retval boolean This return value says if the operation is allowed or not.
 */
bool iNemoGyroSetConfig(iNemoData * pData, uint8_t cParameter, uint8_t* pcValue)
{
  bool isAllowed=FALSE;
  
#ifdef _GYRO
  
  switch(cParameter)
  {
  case 0x00: /* set ODR */
    {
      GyroOutputDataRate xOdr,xOdrVerify;
      
      xOdr = (GyroOutputDataRate)(pcValue[0]<<4);
  
      L3gd20SetDataRate(xOdr);    
      
      xOdrVerify = L3gd20GetEnumDataRate();
      
      /* check if the command has been executed */
      if(xOdr == xOdrVerify)
        /* if yes  send ack  */
        isAllowed=TRUE;
      else
        /* else send nack  */
        isAllowed=FALSE;
      
      break;
    }
  case 0x01: /*  set FS */
    /* Set Full scale*/
    
    if(pcValue[0] < 3)
    {
      
      GyroFullScale xFullScale, xFullScaleVerify;
      
      switch(pcValue[0]){
      case 0:
        xFullScale = L3G_FS_250_DPS;
        iNEMO_SensitivityGyroX(pData)= L3G_Sensitivity_250dps;
        iNEMO_SensitivityGyroY(pData)= L3G_Sensitivity_250dps;
        iNEMO_SensitivityGyroZ(pData)= L3G_Sensitivity_250dps;
        break;
      case 1:
        xFullScale = L3G_FS_500_DPS;
        iNEMO_SensitivityGyroX(pData)= L3G_Sensitivity_500dps;
        iNEMO_SensitivityGyroY(pData)= L3G_Sensitivity_500dps;
        iNEMO_SensitivityGyroZ(pData)= L3G_Sensitivity_500dps;
        break;
      case 2:
        xFullScale = L3G_FS_2000_DPS;
        iNEMO_SensitivityGyroX(pData)= L3G_Sensitivity_2000dps;
        iNEMO_SensitivityGyroY(pData)= L3G_Sensitivity_2000dps;
        iNEMO_SensitivityGyroZ(pData)= L3G_Sensitivity_2000dps;
        break;
        
      }
      
      L3gd20SetFullScale(xFullScale);
      
      xFullScaleVerify = L3gd20GetEnumFullScale();
      
      /* check if the command has been executed */
      if(xFullScale == xFullScaleVerify)
        /* if yes  send ack  */
        isAllowed=TRUE;
      else
        /* else send nack  */
        isAllowed=FALSE;
    }
    else
      /* else send nack  */
      isAllowed=FALSE;
    
    break;
    
    case 0x02:/* set HPF */
      {
        L3GFilterInit xHighPassFilter;

        uint8_t temp=pcValue[0];
        /*
        pcValue[0] = | RFU |RFU |Filter Enable/Disable |REF ENABLE/DISABLE |HP3 |HP2 | HP1 | HP0
        */
        if((temp & 0x20) == 0x20)  /* check filter ENABLE command */
        {

          xHighPassFilter.xHPF=L3G_ENABLE;
          xHighPassFilter.cHPFCutOff = (pcValue[0] & 0x0F);

          if((temp & 0x10) == 0x10)  /* check if reference is enabled */
          {
            xHighPassFilter.xHPF_Mode = L3G_HPFM_REFERENCE;
            xHighPassFilter.cHPFReference = pcValue[1];

            isAllowed=TRUE;
          }
          else /* reference is disabled */
          {
            xHighPassFilter.xHPF_Mode = L3G_HPFM_NORMAL;
            xHighPassFilter.cHPFReference = 0;

            isAllowed=TRUE;
          }

        }
        else /* filter disable*/
        {
          xHighPassFilter.xHPF = L3G_DISABLE;
          xHighPassFilter.cHPFCutOff =0;
          xHighPassFilter.xHPF_Mode = L3G_HPFM_NORMAL;
          xHighPassFilter.cHPFReference = 0;
          
          isAllowed=TRUE;
        }

        L3gxFilterConfig(&xHighPassFilter);

      }
    
    break;
    
    case 0x03: /*  set x axis offset*/
      iNEMO_OffGyroX(pData)=((int16_t)pcValue[0] << 8) + pcValue[1];
      isAllowed=TRUE;
      
      break;
      
    case 0x04: /* set y axis offset*/
      iNEMO_OffGyroY(pData)=((int16_t)pcValue[0] << 8) + pcValue[1];
      isAllowed=TRUE;
   
      break;
      
    case 0x05: /* set z axis offset*/
      iNEMO_OffGyroZ(pData)=((int16_t)pcValue[0] << 8) + pcValue[1];
      isAllowed=TRUE;
    
      break;
      
    case 0x06: /* set x axis scale factor */
      iNEMO_GainGyroX(pData)=(float)(((uint16_t)pcValue[0] << 8) + pcValue[1])/1000;
      isAllowed=TRUE;
      
      break;
      
    case 0x07: /* set y axis scale factor */
      iNEMO_GainGyroY(pData)=(float)(((uint16_t)pcValue[0] << 8) + pcValue[1])/1000;
      isAllowed=TRUE;
     
      break;
    
    case 0x08: /* set z axis scale factor */   
      iNEMO_GainGyroZ(pData)=(float)(((uint16_t)pcValue[0] << 8) + pcValue[1])/1000;
      isAllowed=TRUE;
      
      break;
  }
  
#endif
  
  return isAllowed;
  
}


/**
 * @brief Change a  configuration parameter of a iNEMO pressure sensor.
 * @param pData : pointer to iNemoData structure.
 * @param cParameter : paramenter to change  <ul>
 * <li>0x00 - Output data rate</li>
 * <li>0x01 - Offset </li>
 * <li>0x02 - Scale Factor </li>
 * </ul>
 * @param pcValue pointer to the new value to set.
 * @retval boolean This return value says if the operation is allowed or not.
 */
bool iNemoPressureSetConfig(iNemoData * pData, uint8_t cParameter, uint8_t* pcValue)
{
  bool isAllowed=FALSE;
 
#ifdef _PRESS
  
  switch(cParameter)
  {

  case  0x00:  /*  ODR */
    if(pcValue[0]<0x04)
    {
      LPSOutputDataRate xPressOdr;
      
      switch(pcValue[0])
      {
      case 0x00:
        xPressOdr = ODR_P_1HZ_T_1HZ;
        break;
      case 0x01:
        xPressOdr = ODR_P_7HZ_T_7HZ;
        break;
      case 0x02:
        xPressOdr = ODR_P_12_5HZ_T_12_5HZ;
        break;
      case 0x03:
        xPressOdr = ODR_P_25HZ_T_25HZ;
        break;
      }
      Lps331apSetDataRate(xPressOdr);
      
      isAllowed=TRUE;
    }
    else
    {
      isAllowed=FALSE;
    }
    break;

  case 0x01: /* set pressure offset */
    iNEMO_OffPress(pData)=((int16_t)pcValue[0] << 8) + pcValue[1];
    isAllowed=TRUE;
   
    break;
  
  case 0x02: /* set pressure scale factor */
    iNEMO_GainPress(pData)=(float)(((uint16_t)pcValue[0] << 8) + pcValue[1])/1000;
    isAllowed=TRUE;
    
    break;
  }
#endif
  
  return isAllowed;
}


/**
 * @brief  Change a  configuration parameter of a iNEMO temerature sensor
 * @param pdata : pointer to iNemoData structure
 * @param cParameter : paramenter to change  <ul>
 * <li>0x00 - Offset </li>
 * <li>0x01 - Scale Factor </li>
 * </ul>
 * @param pcValue pointer to the new value to set.
 * @retval boolean This return value says if the operation is allowed or not.
 */
bool iNemoTempSetConfig( iNemoData * pdata, uint8_t cParameter, uint8_t* pcValue)
{
  bool isAllowed=FALSE;
  
#ifdef _TEMP
  
  switch(cParameter)
  {

  case 0x00: /* set temperature offset*/
    iNEMO_OffTemp(pdata)=((int16_t)pcValue[0] << 8) + pcValue[1];
    isAllowed=TRUE;
    
    break;
    
  case 0x01: /* set temperature scale factor*/
    iNEMO_GainTemp(pdata)=(float)(((uint16_t)pcValue[0] << 8) + pcValue[1])/1000;
    isAllowed=TRUE;
    
    break;

  }
  
#endif
  
  return isAllowed;
}


/**
 * @brief  Get a configuration parameter of a iNEMO sensor
 * @param pData : pointer to iNemoData structure
 * @param cSensor : the sensor type (0x00 = Accelerometer; 0x01 = Gyroscope; 0x02 Pitch/Roll Gyro; 0x03 Yaw Gyro; 0x04 = Pressure; 0x05 Temp)
 * @param cParameter : paramenter to get
 * @param pcValue pointer to the value to get.
 * @retval boolean This return value says if the operation is allowed or not.
 */
bool iNemoGetSensorParam(iNemoData * pData, uint8_t cSensor, uint8_t cParameter, uint8_t* pcValue)
{
  bool isAllowed =FALSE;

  switch(cSensor)
  {
  case iNEMO_ACC_ID :  /* Accelerometer */
    isAllowed = iNemoAccGetConfig(pData, cParameter, pcValue);
    break;

  case iNEMO_MAG_ID :  /* Magnetometer */
    isAllowed = iNemoMagGetConfig(pData,cParameter, pcValue);
    break;

  case iNEMO_GYRO_ID  :   /* Gyro */
    isAllowed = iNemoGyroGetConfig(pData,cParameter, pcValue);
    break;

  case iNEMO_PRESS_ID  :  /*  Pressure Sensor*/
    isAllowed = iNemoPressureGetConfig(pData,cParameter, pcValue);
    break;

  case iNEMO_TEMP_ID  : /* Temperature Sensor*/
    isAllowed = iNemoTempGetConfig(pData,cParameter, pcValue);
    break;
    }
  
   return  isAllowed;
  
}


/**
 * @brief  Get a configuration parameter of a iNEMO accelerometer
 * @param pdata : pointer to iNemoData structure
 * @param cParameter : paramenter to change  <ul>
 * <li>0x00 - Output data rate</li>
 * <li>0x01 - Full Scale </li>
 * <li>0x02 - HPF  </li>
 * <li>0x03 - Offset X </li>
 * <li>0x04 - Offset Y </li>
 * <li>0x05 - Offset Z </li>
 * <li>0x06 - Scale Factor X </li>
 * <li>0x07 - Scale Factor Y </li>
 * <li>0x08 - Scale Factor Z </li>
 * <li>0xFF - Sensor Name </li>
 * </ul>
 * @param pcValue pointer to the new value to get.
 * @retval boolean This return value says if the operation is allowed or not.
 */
bool iNemoAccGetConfig(iNemoData* pdata, uint8_t cParameter, uint8_t* pcValue)
  {
    bool isAllowed=FALSE;
    
#ifdef _6X
    
    /* pValue temp[0]=lengh of payload, temp[1]=sensor,  temp[2]=parameter, temp[3]=payload1, temp[4]=payload2(if any)*/
    pcValue[1]=0x00;

    switch(cParameter)
    {
    case 0x00: /* get ODR */
      {
      AccOutputDataRate xOdr;

      pcValue[0]=3;
      pcValue[2]=0x00;

      xOdr = Lsm303dlhcAccGetEnumDataRate();

      switch(xOdr)
      {
      case LSM_ODR_1_HZ:
        pcValue[3] = 0x00;
        break;
      case LSM_ODR_10_HZ:
        pcValue[3] = 0x01;
        break;
      case LSM_ODR_25_HZ:
        pcValue[3] = 0x02;
        break;
      case LSM_ODR_50_HZ:
        pcValue[3] = 0x03;
        break;
      case LSM_ODR_100_HZ:
        pcValue[3] = 0x04;
        break;
      case LSM_ODR_200_HZ:
        pcValue[3] = 0x05;
        break;
      case LSM_ODR_400_HZ:
        pcValue[3] = 0x06;
        break;
      case LSM_ODR_1620_HZ:
        pcValue[3] = 0x07;
        break;
      case LSM_ODR_1344_HZ:
        pcValue[3] = 0x08;
        break;
      }
      isAllowed=TRUE;

      }
      break;
    case  0x01:  /* get Full scale*/
      {
        AccFullScale xFullScale;
        pcValue[0]=3;
        pcValue[2]=0x1;

        xFullScale=Lsm303dlhcAccGetEnumFullScale();

        switch(xFullScale)
        {
        case LSM_FS_2G:
          pcValue[3]=0;
          break;
        case LSM_FS_4G:
          pcValue[3]=1;
          break;
        case LSM_FS_8G:
          pcValue[3]=2;
          break;
        case LSM_FS_16G:
          pcValue[3]=3;
          break;
        }

        isAllowed=TRUE;
      }

      break;

    case  0x02:  /* HPF */
      /*
      return:
      temp[0]=payload lenght
      temp[1] = | RFU |RFU |Filter Enable/Disable |REF ENABLE/DISABLE |HP1 |HP0 | ODR1 |ODR0
      temp[2] = referemce value
      */
      {
      AccOutputDataRate xOdr;
      LSMAccFilterInit xFilterInfo;

      pcValue[0]=4;
      pcValue[2]=0x02;
      pcValue[3]=0x00;

      xOdr = Lsm303dlhcAccGetEnumDataRate();

      switch(xOdr)
      {
      case LSM_ODR_1_HZ:
        pcValue[3] = 0x00;
        break;
      case LSM_ODR_10_HZ:
        pcValue[3] = 0x01;
        break;
      case LSM_ODR_25_HZ:
        pcValue[3] = 0x02;
        break;
      case LSM_ODR_50_HZ:
        pcValue[3] = 0x03;
        break;
      case LSM_ODR_100_HZ:
        pcValue[3] = 0x04;
        break;
      case LSM_ODR_200_HZ:
        pcValue[3] = 0x05;
        break;
      case LSM_ODR_400_HZ:
        pcValue[3] = 0x06;
        break;
      case LSM_ODR_1620_HZ:
        pcValue[3] = 0x07;
        break;
      case LSM_ODR_1344_HZ:
        pcValue[3] = 0x08;
        break;
      }

      Lsm303dlhcAccFilterGetInfo(&xFilterInfo);

      if(xFilterInfo.xHPF)
        pcValue[3] |= 0x20;

      pcValue[3] |= (uint8_t)((xFilterInfo.xHPFCutOff)>>2);

      if(xFilterInfo.xHPF_Mode == LSM_HPFM_REFERENCE)
        pcValue[3] |= 0x10;

      pcValue[4] = xFilterInfo.cHPFReference;


      isAllowed=TRUE;
      break;
      }
    case 0x03: /* get x axis offset*/
      /*
      return:
      temp[0] = payload lenght
      temp[3] = offset MSB
      temp[4] = offset LSB
      */
      pcValue[0]=4;
      pcValue[2]=0x03;
      s16_to_u8_buffer(&iNEMO_OffAccX(pdata), &pcValue[3]);
      isAllowed=TRUE;
      break;

    case 0x04: /* get y axis offset*/
     /*
      return:
      temp[0]=payload lenght
      temp[3] = offset MSB
      temp[4] = offset LSB
      */
      pcValue[0]=4;
      pcValue[2]=0x04;
      s16_to_u8_buffer(&iNEMO_OffAccY(pdata), &pcValue[3]);
      isAllowed=TRUE;
      break;

    case 0x05: /* get z axis offset*/
      /*
      return:
      temp[0]=payload lenght
      temp[3] = offset MSB
      temp[4] = offset LSB
      */
      pcValue[0]=4;
      pcValue[2]=0x05;
      s16_to_u8_buffer(&iNEMO_OffAccZ(pdata), &pcValue[3]);
      isAllowed=TRUE;
      break;
      
   case 0x06: /* get x axis gain*/
    pcValue[0]=4;
    pcValue[2]=0x06;
    uint16_t nGain;
    
    nGain=(uint16_t)(iNEMO_GainAccX(pdata)*1000.0);
    
    u16_to_u8_buffer(&nGain, &pcValue[3]);
    
    isAllowed=TRUE;
    break;
    
    case 0x07: /* Get y axis gain*/
    {
      pcValue[0]=4;
      pcValue[2]=0x07;
      uint16_t nGain;
      
      nGain=(uint16_t)(iNEMO_GainAccY(pdata)*1000.0);
      
      u16_to_u8_buffer(&nGain, &pcValue[3]);
      
      isAllowed=TRUE;
      
    }
    break;  
      
    case 0x08: /* get z axis gain*/
      {
        pcValue[0]=4;
        pcValue[2]=0x08;
        uint16_t nGain;
        
        nGain=(uint16_t)(iNEMO_GainAccZ(pdata)*1000.0);
        
        u16_to_u8_buffer(&nGain, &pcValue[3]);
        
        isAllowed=TRUE;
      }
      break;
      
    case  0xFF:  /* get sensor name */
      strcpy((char*)&pcValue[3],"LSM303DLHC");
      
      pcValue[0]=strlen((char*)&pcValue[3])+2;
      pcValue[2]=0xFF;
      
      isAllowed=TRUE; 
      break;
    }
    
#endif
    
  return isAllowed;

}


/**
 * @brief  Get a configuration parameter of a iNEMO magnetometer.
 * @param pdata : pointer to iNemoData structure
 * @param cParameter : paramenter to change  <ul>
 * <li>0x00 - Output data rate</li>
 * <li>0x01 - Full Scale </li>
 * <li>0x02 - MAGN operationg mode </li>
 * <li>0x03 - Offset X </li>
 * <li>0x04 - Offset Y </li>
 * <li>0x05 - Offset Z </li>
 * <li>0x06 - Scale Factor X </li>
 * <li>0x07 - Scale Factor Y </li>
 * <li>0x08 - Scale Factor Z </li>
 * <li>0xFF - Sensor Name </li>
 * </ul>
 * @param pcValue: pointer to the new value to get.
 * @retval boolean This return value says if the operation is allowed or not.
 */
bool iNemoMagGetConfig(iNemoData* pdata, uint8_t cParameter,  uint8_t* pcValue)
{
  bool isAllowed=FALSE;
  
#ifdef _6X
  /*
  return:
  pcValue[0]=payload lenght
  pcValue[1] = sensor
  pcValue[2] = parameter
  pcValue[3] = payload1
  */
  pcValue[1]=0x01;
  switch(cParameter)
  {
  case 0x00: /* Get ODR */
    {
      MagOutputDataRate xOdr;
      pcValue[0]=3;
      pcValue[2]=0x00;

      xOdr = Lsm303dlhcMagGetEnumDataRate();

      switch(xOdr)
      {
      case LSM_ODR_0_75_HZ:
        pcValue[3]=0;
        break;
      case LSM_ODR_1_5_HZ:
        pcValue[3]=1;
        break;
      case LSM_ODR_3_0_HZ:
        pcValue[3]=2;
        break;
      case LSM_ODR_7_5_HZ:
        pcValue[3]=3;
        break;
      case LSM_ODR_15_HZ:
        pcValue[3]=4;
        break;
      case LSM_ODR_30_HZ:
        pcValue[3]=5;
        break;
      case LSM_ODR_75_HZ:
        pcValue[3]=6;
        break;
      case LSM_ODR_220_HZ:
        pcValue[3]=7;
        break;
      }

      isAllowed=TRUE;
    }
    break;

  case  0x01:  /* Get Full scale*/
    {
      MagFullScale xFullScale;
      
      pcValue[0]=3;
      pcValue[2]=0x01;

      xFullScale = Lsm303dlhcMagGetEnumFullScale();

      switch(xFullScale)
      {
      case LSM_FS_1_3_GA:
        pcValue[3]=1;
        break;
        
      case LSM_FS_1_9_GA:
        pcValue[3]=2;
        break;
      case LSM_FS_2_5_GA:  
        pcValue[3]=3;
        break;
        
      case LSM_FS_4_0_GA:
        pcValue[3]=4;
        break;
        
      case LSM_FS_4_7_GA:
        pcValue[3]=5;
        break;
  
      case LSM_FS_5_6_GA:
        pcValue[3]=6;
        break;
        
      case LSM_FS_8_1_GA:
        pcValue[3]=7;
        break;
        
      }

      isAllowed=TRUE;
    }
    break;

  case  0x02:  /* Get Magn operating mode*/
    pcValue[0]=3;
    pcValue[2]=0x02;
    /* Not available on LSM303DLHC */
    pcValue[3]= 0x00;
    isAllowed=TRUE;
    break;

  case 0x03: /* Get x axis offset*/
    pcValue[0]=4;
    pcValue[2]=0x03;
    s16_to_u8_buffer(&iNEMO_OffMagnX(pdata), &pcValue[3]);
    isAllowed=TRUE;
    break;

  case 0x04: /* Get y axis offset*/
    pcValue[0]=4;
    pcValue[2]=0x04;
    s16_to_u8_buffer(&iNEMO_OffMagnY(pdata), &pcValue[3]);
    isAllowed=TRUE;
    break;

  case 0x05: /* Get z axis offset*/
    pcValue[0]=4;
    pcValue[2]=0x05;
    s16_to_u8_buffer(&iNEMO_OffMagnZ(pdata), &pcValue[3]);
    isAllowed=TRUE;
    break;
    
    
  case 0x06: /* Get x axis gain*/
    {
      pcValue[0]=4;
      pcValue[2]=0x06;
      uint16_t nGain;
      
      nGain=(uint16_t)(iNEMO_GainMagnX(pdata)*1000.0);
      
      u16_to_u8_buffer(&nGain, &pcValue[3]);
      
      isAllowed=TRUE;
    }
    break;
    
    case 0x07: /* Get y axis gain*/
      {
        pcValue[0]=4;
        pcValue[2]=0x07;
        uint16_t nGain;
        
        nGain=(uint16_t)(iNEMO_GainMagnY(pdata)*1000.0);
        
        u16_to_u8_buffer(&nGain, &pcValue[3]);
        
        isAllowed=TRUE;
      }
      break;
      
    case 0x08: /* Get z axis gain*/
      {
        pcValue[0]=4;
        pcValue[2]=0x08;
        uint16_t nGain;
        
        nGain=(uint16_t)(iNEMO_GainMagnZ(pdata)*1000.0);
        
        u16_to_u8_buffer(&nGain, &pcValue[3]);
        
        isAllowed=TRUE;
      }
      break;
      
  case  0xFF:  /* get sensor name */
    strcpy((char*)&pcValue[3],"LSM303DLHC");
    
    pcValue[0]=strlen((char*)&pcValue[3])+2;
    pcValue[2]=0xFF;
    
    isAllowed=TRUE;
    
    break;
  }
  
#endif
  
  return isAllowed;
  
}


/**
 * @brief  Get a configuration parameter of a iNEMO gyroscope.
 * @param pdata : pointer to iNemoData structure
 * @param cParameter : paramenter to change  <ul>
 * <li>0x00 - Output data rate</li>
 * <li>0x01 - Full Scale </li>
 * <li>0x02 - HPF  </li>
 * <li>0x03 - Offset X </li>
 * <li>0x04 - Offset Y </li>
 * <li>0x05 - Offset Z </li>
 * <li>0x06 - Scale Factor X </li>
 * <li>0x07 - Scale Factor Y </li>
 * <li>0x08 - Scale Factor Z </li>
 * <li>0xFF - Sensor Name </li>
 * </ul>
 * @param pcValue: pointer to the parameter to get.
 * @retval boolean This return value says if the operation is allowed or not.
 */
bool iNemoGyroGetConfig(iNemoData* pdata, uint8_t cParameter,  uint8_t* pcValue)
{
    
  bool isAllowed=FALSE;
  
#ifdef _GYRO
  
  pcValue[1]=0x02;
  /*
      return:
      temp[0] = payload lenght
      temp[1] = sensor
      temp[2] = parameter
      temp[3] = payload1
      temp[4] = payload2 (if any)
  */
  
  switch(cParameter)
  {
  case 0x00: 
    /*  Get ODR */
    {
      GyroOutputDataRate xOdr;
     
      pcValue[0]=3;
      pcValue[2]=0x00;
        
      
      xOdr = L3gd20GetEnumDataRate();
      
      pcValue[3]=((uint8_t)xOdr)>>4;
      
      /* to send ack  */
      isAllowed=TRUE;
        
      break;
    }
  case 0x01: 
    /* Get Full scale*/
    {
      GyroFullScale xFullScale;
      
      pcValue[0]=3;
      pcValue[2]=0x01;
      
      
      xFullScale=L3gd20GetEnumFullScale();
      
      pcValue[3]=((uint8_t)xFullScale)>>4;

      /* to send ack  */
      isAllowed=TRUE;
      
      
      break;
    }
        
    case  0x02:  
      /* Get HPF parameters */
      /*
      return:
      temp[0]=payload lenght
      temp[1] = | RFU |RFU |Filter Enable/Disable |REF ENABLE/DISABLE |HP1 |HP0 | ODR1 |ODR0
      temp[2] = referemce value
      */
      {
      L3GFilterInit xFilterInfo;

      pcValue[0]=4;
      pcValue[2]=0x02;


      L3gd20FilterGetInfo(&xFilterInfo);
      
      pcValue[3] = (uint8_t)xFilterInfo.cHPFCutOff;
      
      if(xFilterInfo.xHPF)
        pcValue[3] |= 0x20;


      if(xFilterInfo.xHPF_Mode == L3G_HPFM_REFERENCE)
        pcValue[3] |= 0x10;

      pcValue[4] = xFilterInfo.cHPFReference;


      isAllowed=TRUE;
      break;
      }
      
    case 0x03: 
      /*  set x axis offset*/
      /*
      temp[3] = offset MSB
      temp[4] = offset LSB
      */
      pcValue[0]=4;
      pcValue[2]=0x03;
      s16_to_u8_buffer(&iNEMO_OffGyroX(pdata), &pcValue[3]);
      isAllowed=TRUE;

      break;
      
    case 0x04:
      /*  set y axis offset*/
      /*
      temp[3] = offset MSB
      temp[4] = offset LSB
      */
      pcValue[0]=4;
      pcValue[2]=0x04;
      s16_to_u8_buffer(&iNEMO_OffGyroY(pdata), &pcValue[3]);
      isAllowed=TRUE;

      break;
      
    case 0x05:
      /*  set z axis offset*/
      /*
      temp[3] = offset MSB
      temp[4] = offset LSB
      */
      pcValue[0]=4;
      pcValue[2]=0x05;
      s16_to_u8_buffer(&iNEMO_OffGyroZ(pdata), &pcValue[3]);
      isAllowed=TRUE;

      break;
      
  case 0x06: /* get x axis scale factor */
    pcValue[0]=4;
    pcValue[2]=0x06;
    uint16_t nGain;
    
    nGain=(uint16_t)(iNEMO_GainGyroX(pdata)*1000.0);
    
    u16_to_u8_buffer(&nGain, &pcValue[3]);
    
    isAllowed=TRUE;
    break;
    
  case 0x07: /* get y axis scale factor */
    {
      pcValue[0]=4;
      pcValue[2]=0x07;
      uint16_t nGain;
      
      nGain=(uint16_t)(iNEMO_GainGyroY(pdata)*1000.0);
      
      u16_to_u8_buffer(&nGain, &pcValue[3]);
      
      isAllowed=TRUE;
    }
      break;
    
  case 0x08: /* get z axis scale factor */
    {
      pcValue[0]=4;
      pcValue[2]=0x08;
      uint16_t nGain;
      
      nGain=(uint16_t)(iNEMO_GainGyroZ(pdata)*1000.0);
      
      u16_to_u8_buffer(&nGain, &pcValue[3]);
      
      isAllowed=TRUE;
    }
    break;
      
      
  case  0xFF:  /* get sensor name */
      strcpy((char*)&pcValue[3],"L3GD20");
    
      pcValue[0]=strlen((char*)&pcValue[3])+2;
      pcValue[2]=0xFF;
    
      isAllowed=TRUE;
      break;
  }
  
#endif
  
  return isAllowed;
}


/**
 * @brief  Get a configuration parameter of a iNEMO Pressure Sensor
 * @param pdata : pointer to iNemoData structure
 * @param cParameter : paramenter to change  <ul>
 * <li>0x00 - ODR </li>
 * <li>0x01 - Offset </li>
 * <li>0x02 - Scale Factor </li>
 * <li>0xFF - Sensor Name </li>
 * </ul>
* @param pcValue: pointer to the value to get.
 * @retval boolean This return value says if the operation is allowed or not.
 */
bool iNemoPressureGetConfig( iNemoData * pdata, uint8_t cParameter, uint8_t* pcValue)
{
  bool isAllowed=FALSE;

#ifdef _PRESS
  
  /*
  return:
  temp[0]=payload lenght
  temp[1] = sensor
  temp[2] = parameter
  temp[3] = payload1
  temp[4] = payload2
  */
  
  pcValue[1]=0x04;
  switch(cParameter)
  {
  case  0x00:  /* ODR */
    {
      LPSOutputDataRate xPressOdr;
      
      pcValue[0]= 3;
      pcValue[2]= 0x00;
      xPressOdr=Lps331apGetDataRate();
      
      switch(xPressOdr)
      {
      case ODR_P_1HZ_T_1HZ:
        pcValue[3] = 0;
        break;
      case ODR_P_7HZ_T_7HZ:
        pcValue[3] = 1;
        break;
      case ODR_P_12_5HZ_T_12_5HZ:
        pcValue[3] = 2;
        break;  
      case ODR_P_25HZ_T_25HZ:
        pcValue[3] = 3;
        break; 
      }
      isAllowed=TRUE;
    }
      
    break;

  case 0x01: /* get pressure offset*/
      pcValue[0]=4;
      pcValue[2]=0x01;
      s16_to_u8_buffer(&iNEMO_OffPress(pdata), &pcValue[3]);
      isAllowed=TRUE;
      break;
      
  case 0x02: /* get scale factor */
    {
      pcValue[0]=4;
      pcValue[2]=0x02;
      uint16_t nGain;
      
      nGain=(uint16_t)(iNEMO_GainPress(pdata)*1000.0);
      
      u16_to_u8_buffer(&nGain, &pcValue[3]);
      
      isAllowed=TRUE;
    }
    break;
    
  case  0xFF:  /* get sensor name */
      strcpy((char*)&pcValue[3],"LPS331AP");
      
      pcValue[0]=strlen((char*)&pcValue[3])+2;
      pcValue[2]=0xFF;
    
      isAllowed=TRUE;
      break;
  }
  
#endif
  
  return isAllowed;
  
}


/**
 * @brief  Get a configuration parameter of a iNEMO Temperature Sensor
 * @param pdata : pointer to iNemoData structure
 * @param cParameter : paramenter to change  <ul>
 * <li>0x00 - Offset  </li>
 * <li>0x01 - Scale Factor </li>
 * <li>0xFF - Sensor Name </li>
 * </ul>
 * @param pcValue pointer to the value to get.
 * @retval boolean This return value says if the operation is allowed or not.
 */
bool iNemoTempGetConfig( iNemoData * pdata, uint8_t cParameter, uint8_t* pcValue)
{
    bool isAllowed=FALSE;
    
#ifdef _TEMP
    
    /*
      return:
      temp[0]=payload lenght
      temp[1] = sensor
      temp[2] = paramter
      temp[3] = payload1
      temp[4] = payload2
      */
    pcValue[1]=0x05;
  switch(cParameter)
  {
  case 0x00: /* get temperature offset*/
      pcValue[0]=4;
      pcValue[2]=0x00;
      s16_to_u8_buffer(&iNEMO_OffTemp(pdata), &pcValue[3]);
      isAllowed=TRUE;
      break;
      
  case 0x01: /* get gain*/
    pcValue[0]=4;
    pcValue[2]=0x01;
    uint16_t nGain;
    
    nGain=(uint16_t)(iNEMO_GainTemp(pdata)*1000.0);
    
    u16_to_u8_buffer(&nGain, &pcValue[3]);
    
    isAllowed=TRUE;
    break;
    
  case  0xFF:  /* get sensor name */
    {
      strcpy((char*)&pcValue[3],"LSM303DLHC");
      
      pcValue[0]=strlen((char*)&pcValue[3])+2;
      pcValue[2]=0xFF;
      
      isAllowed=TRUE;
    }
    
    break;
  }
#endif
  
  return isAllowed;
  
}


/**
 * @brief Change a a configuration parameter of a iNEMO sensor
 * @param pdata : poiter to iNemoData structure
 * @param cSensor : the sensor type (0x00 = Accelerometer; 0x01 = Magnetometer ; 0x02 = Gyroscope; 0x04 = Pressure; 0x05 Temp)
 * @param cParameter : paramenter to change
 * @param pcValue: pointer to the value to restore.
 */
bool iNemoRestoreDefaultParam(iNemoData * pdata, uint8_t cSensor, uint8_t cParameter, uint8_t* pcValue)
{
  bool isAllowed =FALSE;
  switch(cSensor)
  {
  case 0x00 :  /*  Accelerometer */
    switch(cParameter)
    {
    case 0x00: /*  set ODR */
      pcValue[0]=3;
      pcValue[1]=0;
      isAllowed = iNemoAccSetConfig(pdata, cParameter, &pcValue[1]);
      break;

    case 0x01: /* set FS */
      pcValue[0]=3;
      pcValue[1]=0;
      isAllowed = iNemoAccSetConfig(pdata, cParameter, &pcValue[1]);
      break;

    case 0x02: /*!<  set HPS */
      pcValue[0]=4;
      pcValue[1]=0;
      isAllowed = iNemoAccSetConfig(pdata, cParameter, &pcValue[1]);
      break;

    case 0x03: /* Set x offset*/
      pcValue[0]=4;
      pcValue[1]=0;
      pcValue[2]=0;
      isAllowed = iNemoAccSetConfig(pdata, cParameter, &pcValue[1]);
      break;

    case 0x04: /*  Set y offset*/
      pcValue[0]=4;
      pcValue[1]=0;
      pcValue[2]=0;
      isAllowed = iNemoAccSetConfig(pdata, cParameter, &pcValue[1]);
      break;

    case 0x05: /*  Set z offset*/
      pcValue[0]=4;
      pcValue[1]=0;
      pcValue[2]=0;
      isAllowed = iNemoAccSetConfig(pdata, cParameter, &pcValue[1]);
      break;
    }
    break;

  case 0x01 :  /*  Magnetometer */
    switch(cParameter)
    {
    case 0x00: /*  set ODR */
      pcValue[0]=3;
      pcValue[1]=0;
      isAllowed = iNemoMagSetConfig(pdata, cParameter, &pcValue[1]);
      break;

    case 0x01: /*  set ODR */
      pcValue[0]=3;
      pcValue[1]=0;
      isAllowed = iNemoMagSetConfig(pdata, cParameter, &pcValue[1]);
      break;

    case 0x02: /* set Mode */
      pcValue[0]=4;
      pcValue[1]=0;
      isAllowed = iNemoMagSetConfig(pdata, cParameter, &pcValue[1]);
      break;

    case 0x03: /* Set x offset*/
      pcValue[0]=4;
      pcValue[1]=0;
      pcValue[2]=0;
      isAllowed = iNemoMagSetConfig(pdata, cParameter, &pcValue[1]);
      break;

    case 0x04: /* Set y offset*/
      pcValue[0]=4;
      pcValue[1]=0;
      pcValue[2]=0;
      isAllowed = iNemoMagSetConfig(pdata, cParameter, &pcValue[1]);
      break;

    case 0x05: /* Set z offset*/
      pcValue[0]=4;
      pcValue[1]=0;
      pcValue[2]=0;
      isAllowed = iNemoMagSetConfig(pdata, cParameter, &pcValue[1]);
      break;
    case 0x06:
      iNEMO_GainMagnX(pdata)=1;
      iNEMO_GainMagnY(pdata)=1;
      iNEMO_GainMagnZ(pdata)=1;
      isAllowed = TRUE;
      break;
    }
    break;

  case 0x02  :  /* Gyro */
    switch(cParameter)
    {
    case 0x00: /* set ODR */
      pcValue[0]=3;
      pcValue[1]=0;
      isAllowed = iNemoGyroSetConfig(pdata, cParameter,&pcValue[1]);
      break;

    case 0x01: /* set FS */
      pcValue[0]=3;
      pcValue[1]=0;
      isAllowed = iNemoGyroSetConfig(pdata, cParameter, &pcValue[1]);
      break;

    case 0x02: /* set HPS */
      pcValue[0]=4;
      pcValue[1]=0;
      isAllowed = iNemoGyroSetConfig(pdata, cParameter, &pcValue[1]);
      break;

    case 0x03: /* Set x offset*/
      pcValue[0]=4;
      pcValue[1]=0;
      pcValue[2]=0;
      isAllowed = iNemoGyroSetConfig(pdata, cParameter, &pcValue[1]);
      break;

    case 0x04: /* Set y offset*/
      pcValue[0]=4;
      pcValue[1]=0;
      pcValue[2]=0;
      isAllowed = iNemoGyroSetConfig(pdata, cParameter, &pcValue[1]);
      break;

    case 0x05: /* Set z offset*/
      pcValue[0]=4;
      pcValue[1]=0;
      pcValue[2]=0;
      isAllowed = iNemoGyroSetConfig(pdata, cParameter, &pcValue[1]);
      break;
    }
    break;

  case 0x04  :  /* Pressure Sensor*/
    switch(cParameter)
    {
    case 0x00: /* set ODR */
      pcValue[0]=3;
      pcValue[1]=0;
      isAllowed = iNemoPressureSetConfig(pdata, cParameter, &pcValue[1]);
      break;
    case 0x01: /* Set offset*/
      pcValue[0]=4;
      pcValue[1]=0;
      pcValue[2]=0;
      isAllowed = iNemoPressureSetConfig(pdata, cParameter, &pcValue[1]);
      break;
    }

    break;

  case 0x05: /* Temperature Sensor*/
    switch(cParameter)
    {
    case 0x00: /* set ODR */
      pcValue[0]=4;
      pcValue[1]=0;
      pcValue[2]=0;
      isAllowed = iNemoTempSetConfig(pdata, cParameter, &pcValue[1]);
      break;
    }
    break;
  }
  return  isAllowed;
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

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
