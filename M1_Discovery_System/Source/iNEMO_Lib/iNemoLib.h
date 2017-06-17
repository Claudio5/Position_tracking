/**
  * @file    iNemoLib.h
  * @author  ART Team IMS-Systems Lab
  * @version V2.3.0
  * @date    01 February 2013
  * @brief   This file includes the sensor and features header files in the user application.
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

/* Define to prevent recursive inclusion */
#ifndef __iNEMO_LIB_H
#define __iNEMO_LIB_H

/* includes */
#include "stm32f10x.h"
#include "iNemoConf.h"
#include "utils.h"

#ifdef __cplusplus
 extern "C" {
#endif

#ifndef _6X
  #undef _TEMP
#endif
   
#ifdef AHRS_MOD
  #ifdef AHRS_JUMP
     #include "iNEMO_AHRS_Interface.h"
  #else
     #include "iNEMO_AHRS.h"
  #endif
#endif

#ifdef COMPASS_MOD
#include "iNEMO_Compass.h"
#endif
   
#ifdef _6X
  #include "LSM303DLHC.h"
#endif

#ifdef _GYRO
  #include "L3Gx.h"
#endif

#ifdef _PRESS
#include "LPS331AP.h"
#endif

#ifdef _TEMP
/**
 * @brief Enable the temperature sensor embedded in LSM303DLHC
 */   
  #define LSM_TEMP_ON
#endif

#ifdef _ULED
  #include "iNemoLed.h"
#endif

   
/**
 * @addtogroup iNEMO_Discovery_System
 * @{
 */
   
/**
 * @addtogroup iNEMO_Lib
 * @{
 */

/**
 * @brief HW version APR4
 */ 
#define M1_VER_PC29APR4  0                
/**
 * @brief HW version APR1
 */    
#define M1_VER_PC29APR1  1                
   
/**
 * @defgroup  Sensor_Management              Sensor Management
 * @brief ID and number of parameters
 * @{
 */

/**
 * @brief Accelerometer ID 
 */   
#define iNEMO_ACC_ID            0x00    
   
/**
 * @brief Magnetometer ID 
 */  
#define iNEMO_MAG_ID            0x01 
   
/**
 * @brief Gyroscope ID 
 */  
#define iNEMO_GYRO_ID           0x02 
   
/**
 * @brief Pressure sensor ID 
 */    
#define iNEMO_PRESS_ID          0x04   
   
/**
 * @brief Temperature ID 
 */  
#define iNEMO_TEMP_ID           0x05 

/**
 * @brief Number of parameters for accelerometer 
 */
#define iNEMO_ACC_N_PARS        9     

/**
 * @brief Number of parameters for gyro 
 */
#define iNEMO_GYRO_N_PARS       9    

/**
 * @brief Number of parameters for magnetometer 
 */
#define iNEMO_MAG_N_PARS        9    
   
/**
 * @brief Number of parameters for pressure sensor 
 */ 
#define iNEMO_PRESS_N_PARS      3      
   
/**
 * @brief  Number of parameters for temperature sensor 
 */ 
#define iNEMO_TEMP_N_PARS       2     

/**
 * @brief  Macro to calculate the total sensors parameters number 
 */
#define N_TOT_SAVED_PARS  (iNEMO_ACC_N_PARS+iNEMO_GYRO_N_PARS+iNEMO_MAG_N_PARS+iNEMO_PRESS_N_PARS+iNEMO_TEMP_N_PARS) 
   
/**
* @}
*/

/**
 * @defgroup  Sensor_Sampling              Sensor Sampling
 * @brief Sensor sampling macros. To be used by the GUI layer in order to make abstract the sensor library.
 * @{
 */
   
/**
 * @brief Gyro sensor mapping
 */
#ifdef _GYRO
/**
 * @brief  Mapping of iNemoLibGetGyroRawData function in @ref L3gd20ReadRawData in case Gyroscope is defined
 */   
  #define iNemoLibGetGyroRawData(data)       L3gd20ReadRawData(data)
#else
/**
 * @brief  Mapping of iNemoLibGetGyroRawData function in NULL in case Gyroscope is not defined
 */    
  #define iNemoLibGetGyroRawData(data)  
#endif
   
/**
 * @brief 6 axis (accelerometer and magnetometer) sensor mapping
 */
#ifdef _6X
/**
 * @brief  Mapping of iNemoLibGetAccRawData function for Accelerometer in @ref Lsm303dlhcAccReadRawData in case 6-axis sensor is defined
 */   
  #define iNemoLibGetAccRawData(data)        Lsm303dlhcAccReadRawData(data)
/**
 * @brief  Mapping of iNemoLibGetMagRawData function for Magnetometer in @ref Lsm303dlhcMagReadRawData in case 6-axis sensor is defined
 */   
  #define iNemoLibGetMagRawData(data)        Lsm303dlhcMagReadRawData(data)
#else
/**
 * @brief  Mapping of iNemoLibGetAccRawData function for Accelerometer in NULL in case 6-axis sensor is not defined
 */   
  #define iNemoLibGetAccRawData(data) 
/**
 * @brief  Mapping of iNemoLibGetMagRawData function for Magnetometer in NULL in case 6-axis sensor is not defined
 */   
  #define iNemoLibGetMagRawData(data)        
#endif
   
/**
 * @brief Pressure sensor mapping
 */
#ifdef _PRESS
/**
 * @brief  Mapping of iNemoLibGetPressRawData function in @ref Lps331apReadRawPressure in case pressure sensor is defined
 */
  #define iNemoLibGetPressRawData()      Lps331apReadRawPressure()
#else
/**
 * @brief  Mapping of iNemoLibGetPressRawData function in NULL in case pressure sensor is not defined
 */   
  #define iNemoLibGetPressRawData()      0 
#endif

/**
 * @brief Temperature sensor mapping
 */   
#ifdef _TEMP
/**
 * @brief  Mapping of iNemoLibGetTempRawData function in @ref Lsm303dlhcMagReadRawDataTemp in case temperature sensor is defined
 */   
  #define iNemoLibGetTempRawData()       Lsm303dlhcMagReadRawDataTemp()
#else
/**
 * @brief  Mapping of iNemoLibGetTempRawData function in NULL in case temperature sensor is not defined
 */   
  #define iNemoLibGetTempRawData()       0 
#endif
   
   
/**
* @}
*/

/**
 * @defgroup  Flash_Definitions              Flash Definitions
 * @brief Flash defines. Used to access memory.
 * @{
 */
/**
 * @brief Flash Address for Parameters settings
 */   
#define FLASH_PARS_ADDR        0x0807F800

/**
 * @brief Version of flash data format
 */     
#define FLASH_DATA_VERSION     0x00   
   
/**
 * @brief Flash erased code
 */   
#define FLASH_STATE_ERASED     0x03  
/**
 * @brief Flash filled with data code
 */   
#define FLASH_STATE_READY      0x00  

/**
 * @brief Max Sensor ID to be used by @ref iNemoSensorParsStructAutoInit to auto-build the flash metadata
 */    
#define FLASH_METAD_MAX_SENS_ID       6    
/**
 * @brief Max Parameter ID to be used by @ref iNemoSensorParsStructAutoInit to auto-build the flash metadata
 */     
#define FLASH_METAD_MAX_PAR_ID        10   

/**
 * @}
 */
   
/**
 * @struct DataReadyIRQConf
 * @brief DataReady line parameters grouped in a structure.
 */
typedef struct{
  uint32_t lGpioClock;                  /*!< GPIO Port Clock. */
  GPIO_TypeDef *pxGpioPort;             /*!< GPIO Port. */
  uint16_t nGpioPin;                    /*!< GPIO Pin. */
  uint8_t cGpioPortSource;              /*!< GPIO Port Source. */
  uint8_t cGpioPinSource;               /*!< GPIO Pin Source. */
  uint32_t nExtiLine;                   /*!< GPIO External Line. */
  IRQn_Type nIRQnCh;                    /*!< IRQ Channel. */
} DataReadyIRQConf;

/**
 * @struct iNemoSensorPars
 * @brief Sensor parameters structure used to map these parameters in flash.
 */
typedef struct
{
  uint8_t cIdSensPar;       /*!< Sensor and parameter ID in the format: ID_SENSOR(4bits)|ID_PARAM(4bits). */
  uint8_t cSize;            /*!< Parameter size expressed in bytes. */
} iNemoSensorPars;


/**
 * @struct iNemoData
 * @brief iNemo data structure.
 */
typedef struct
{

  int16_t pnAcc[3];             /*!< Accelerometer XYZ data (raw) */
  int16_t pnGyro[3];            /*!< Gyroscope XYZ data (raw) */
  int16_t pnMag[3];             /*!< Magnetometer XYZ data (raw) */

  int32_t lPress;               /*!< Pressure sensor data (raw) */
  int16_t nTemp;                /*!< Temperature sensor data (raw) */
  
  float fSensitivity[11];       /*!< Sensitivities. Divide the raw in order to have a non-rated measurement.
                                        <br> The order of sensitivities in the array is {Acc[XYZ] - Gyro[XYZ] - Magn[XYZ] - Press - Temp} */
  
  float fScaleFactor[11];       /*!< Scale Factor. The calibrated data is obtained as ((raw_data/sensitivity[i])-nOffset[i])/fScaleFactor[i].
                                        <br> The order of scale factor in the array is {Acc[XYZ] - Gyro[XYZ] - Magn[XYZ] - Press - Temp} */
  int16_t nOffset[11];          /*!< Offset. The calibrated data is obtained as ((raw_data/sensitivity[i])-nOffset[i])/fScaleFactor[i].
                                        <br> The order of offset in the array is {Acc[XYZ] - Gyro[XYZ] - Magn[XYZ] - Press - Temp} */


#if defined(AHRS_MOD) || defined(COMPASS_MOD)
#ifdef AHRS_MOD  
  iNEMO_QUAT          xQuat;         /*!< Quaternions data */
#endif
#ifdef COMPASS_MOD  
  float               fHeading;      /*!< Heading angle data (rad) */
#endif
  iNEMO_SENSORDATA    xSensorData;   /*!< Sensor data to be elaborated by an algorithm. In this structure the user can decide to store preprocessed data (example: exchange axis or multiply by a constant) before an algorithm will process it (i.e. AHRS, COMPASS). */
  iNEMO_EULER_ANGLES  xEulerAngles;  /*!< Euler angles data (rad) */
#endif
  
} iNemoData;


/**
 * @addtogroup iNEMO_Lib_Functions                   iNEMO Library Functions
 * @{
 */
uint8_t iNemoGetVersion(void);
void iNemoHwConfig(void);
#ifdef _6X
void iNemoAccDataReadyConfig(FunctionalState xNewState);
void iNemoMagDataReadyConfig(FunctionalState xNewState);
#endif
#ifdef _GYRO
void iNemoGyroDataReadyConfig(FunctionalState xNewState);
#endif
#ifdef _PRESS
void iNemoPressDataReadyConfig(FunctionalState xNewState);
#endif
void iNemoSensorsConfig(void);
void iNemoSensorParametersInit(iNemoData* data);
void iNemoBoardRecognition(void);
bool iNemoSetSensor(iNemoData* pdata, uint8_t cSensor, uint8_t cParameter, uint8_t cLength, uint8_t* pvalue);
bool iNemoAccSetConfig(iNemoData* pdata, uint8_t cParameter, uint8_t* value);
bool iNemoMagSetConfig(iNemoData* pdata, uint8_t cParameter, uint8_t* pvalue);
bool iNemoGyroSetConfig(iNemoData * pdata, uint8_t cParameter, uint8_t* pvalue);
bool iNemoPressureSetConfig(iNemoData* pdata, uint8_t cParameter, uint8_t* pvalue);
bool iNemoTempSetConfig(iNemoData* pdata, uint8_t cParameter, uint8_t* pvalue);
bool iNemoGetSensorParam(iNemoData* pdata, uint8_t cSensor, uint8_t uparameter, uint8_t* pvalue);
bool iNemoAccGetConfig(iNemoData* pdata, uint8_t cParameter, uint8_t* pvalue);
bool iNemoMagGetConfig(iNemoData* pdata, uint8_t cParameter,  uint8_t* pvalue);
bool iNemoGyroGetConfig(iNemoData* pdata, uint8_t cParameter,  uint8_t* pvalue);
bool iNemoPressureGetConfig(iNemoData* pdata, uint8_t cParameter, uint8_t* pvalue);
bool iNemoTempGetConfig(iNemoData* pdata, uint8_t cParameter, uint8_t* pvalue);

bool iNemoRestoreDefaultParam(iNemoData* pdata, uint8_t cSensor, uint8_t cParameter, uint8_t* pvalue);

void iNemoReadFromFlash(iNemoData* data);
void iNemoSaveToFlash(iNemoData* pData, FlagStatus xKeepFlashMap);
void WriteBufferToFlash(uint32_t lSectStartAddr, uint8_t* pcBuffer, uint8_t cSize);


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
