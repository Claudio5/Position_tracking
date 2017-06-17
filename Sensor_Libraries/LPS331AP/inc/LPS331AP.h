/**
 * @file    LPS331AP.h
 * @author  ART Team IMS-Systems Lab
 * @version V2.3.0
 * @date    12 April 2012
 * @brief   Header for LPS331AP.c file
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



/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LPS331AP_H
#define __LPS331AP_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "HAL_LPS331AP.h"

#include <stdio.h>
#include <math.h>

#ifdef __cplusplus
 extern "C" {
#endif

   
/**
 * @addtogroup Sensor_Libraries           Sensor Libraries
 * @{
 */
   
/**
 * @defgroup LPS331AP   LPS331AP
 * @brief This module contains all the functions to configure the LPS331AP pressure sensor.
 * @details
 * Since this code is platform independent an implementation of the SPI or I2C driver must
 * be provided by the user according to the used platform.
 * Every function makes use of the <i>Lps331apBufferRead</i> and/or <i>Lps331apBufferWrite</i>
 * as low level functions to write bytes through the used digital interface.
 * In order to link and use this code the user should define and export these functions in a header
 * file called "HAL_LPS331AP.h" (included by this module).
 * 
 * @{
 */

/**
 * @defgroup LPS331AP_Exported_Types            LPS331AP Exported Types
 * @{
 */

/**
 * @brief  LPS331AP Functional state. Used to enable or disable a specific option. 
 */   
typedef enum
{
  LPS_DISABLE = 0,                      /*!< Disable */
  LPS_ENABLE = !LPS_DISABLE             /*!< Enable */
}LPSFunctionalState; 
   
   
/**
 * @brief  Pressure Flag status. Used to set/reset the sensor flags.
 */   
typedef enum
{
  LPS_RESET = 0,                        /*!< RESET */
  LPS_SET = !LPS_RESET                  /*!< SET */
}LPSFlagStatus;


/**
 * @brief LPS331AP Output Data Rate
 */   
typedef enum
{
  ODR_P_T_ONE_SHOT        = 0x00,         /*!< Output Data Rate: P - one shot, T - one shot */
  ODR_P_1HZ_T_1HZ         = 0x10,         /*!< Output Data Rate: P - 1Hz, T - 1Hz */
  ODR_P_7HZ_T_1HZ         = 0x20,         /*!< Output Data Rate: P - 7Hz, T - 1Hz */
  ODR_P_12_5HZ_T_1HZ      = 0x30,         /*!< Output Data Rate: P - 12.5Hz, T - 1Hz */
  ODR_P_25HZ_T_1HZ        = 0x40,         /*!< Output Data Rate: P - 25Hz, T - 1Hz */
  ODR_P_7HZ_T_7HZ         = 0x50,         /*!< Output Data Rate: P - 7Hz, T - 7Hz */
  ODR_P_12_5HZ_T_12_5HZ   = 0x60,         /*!< Output Data Rate: P - 12.5Hz, T - 12.5Hz */
  ODR_P_25HZ_T_25HZ       = 0x70,         /*!< Output Data Rate: P - 25Hz, T - 25Hz */
}LPSOutputDataRate;


/**
 * @brief LPS331AP Pressure resolution
 */   
typedef enum
{
  LPS_PRESS_AVG_1          = 0x00,         /*!< Internal average on 1 sample */
  LPS_PRESS_AVG_2          = 0x01,         /*!< Internal average on 2 sample */
  LPS_PRESS_AVG_4          = 0x02,         /*!< Internal average on 4 sample */
  LPS_PRESS_AVG_8          = 0x03,         /*!< Internal average on 8 sample */
  LPS_PRESS_AVG_16         = 0x04,         /*!< Internal average on 16 sample */
  LPS_PRESS_AVG_32         = 0x05,         /*!< Internal average on 32 sample */
  LPS_PRESS_AVG_64         = 0x06,         /*!< Internal average on 64 sample */
  LPS_PRESS_AVG_128        = 0x07,         /*!< Internal average on 128 sample */
  LPS_PRESS_AVG_256        = 0x08,         /*!< Internal average on 256 sample */
  LPS_PRESS_AVG_384        = 0x09,         /*!< Internal average on 384 sample */
  LPS_PRESS_AVG_512        = 0x0A,         /*!< Internal average on 512 sample */
}LPSPressureResolution;


/**
 * @brief LPS331AP Temperature resolution
 */   
typedef enum
{
  LPS_TEMP_AVG_1          = 0x00,         /*!< Internal average on 1 sample */
  LPS_TEMP_AVG_2          = 0x10,         /*!< Internal average on 2 sample */
  LPS_TEMP_AVG_4          = 0x20,         /*!< Internal average on 4 sample */
  LPS_TEMP_AVG_8          = 0x30,         /*!< Internal average on 8 sample */
  LPS_TEMP_AVG_16         = 0x40,         /*!< Internal average on 16 sample */
  LPS_TEMP_AVG_32         = 0x50,         /*!< Internal average on 32 sample */
  LPS_TEMP_AVG_64         = 0x60,         /*!< Internal average on 64 sample */
  LPS_TEMP_AVG_128        = 0x70,         /*!< Internal average on 128 sample */
}LPSTemperatureResolution;



/**
 * @brief LPS331AP Irq list
 */
typedef enum
{
  LPS_GND = 0x00,                       /*!< GND */
  LPS_P_HIGH = 0x01,                    /*!< Pressure High */
  LPS_P_LOW = 0x02,                     /*!< Pressure Low */
  LPS_P_LOW_HIGH = 0x03,                /*!< Pressure Low or Pressure High */
  LPS_DATA_READY = 0x04,                /*!< Data Ready */
  LPS_TRISTATE = 0x07,                  /*!< Tri-State */
}LPSIrqList;


/**
 * @brief LPS331AP Irq pin output configuration
 */
typedef enum
{
  LPS_PP = 0x00,                /*!< Push-Pull */
  LPS_OD = 0x40                 /*!< Open-Drain */
}LPSOutputType;


/**
 * @brief Pressure sensor Irq initialization structure
 */
typedef struct
{
  LPSFunctionalState xIrqActiveLow;             /*!< Enabling/Disabling Irq */
  LPSOutputType xOutType;                       /*!< Push-pull or Open-Drain */
  LPSIrqList xInt1List;                         /*!< List of IRQ on INT1 */
  LPSIrqList xInt2List;                         /*!< List of IRQ on INT2 */
  float fPressThr;                              /*!< Threshold */
  LPSFunctionalState xIrqPressLow;              /*!< Interrupt enabling/disabling on Pressure Low */
  LPSFunctionalState xIrqPressHigh;             /*!< Interrupt enabling/disabling on Pressure High */
}LPSIrqInit;


/**
 * @brief Pressure sensor Init structure definition
 */
typedef struct
{
  LPSOutputDataRate xOutputDataRate;      /*!< Output Data Rate */
  LPSPressureResolution xPresRes;         /*!< Pressure sensor resolution */
  LPSTemperatureResolution xTempRes;      /*!< Temperature sensor resolution */
  LPSFunctionalState xPressureAutoZero;   /*!< Auto zero feature enabled */
  float fPressureRef;                     /*!< Pressure sensor reference value */
  LPSFunctionalState xBDU;                /*!< Block data update */
  LPSFunctionalState xTemperatureSensor;  /*!< Temperature sensor enabling/disabling */         //NEW
}LPS331Init;


/**
 * @brief Pressure sensor data status structure structure
 */
typedef struct
{
  LPSFlagStatus cTempDataAvailable:1;     /*!< Temperature data available bit */
  LPSFlagStatus cPressDataAvailable:1;    /*!< Pressure data available bit */
  uint8_t :2;                             /*!< 2 bits padding */
  LPSFlagStatus cTempDataOverrun:1;       /*!< Temperature data over-run bit */
  LPSFlagStatus cPressDataOverrun:1;      /*!< Pressure data over-run bit */
  uint8_t :2;                             /*!< 2 bits padding */
}LPS331DataStatus;


/**
 * @}
 */    
  
/**
 * @defgroup LPS331AP_Exported_Constants       LPS331AP Exported Constants
 * @{
 */
     

/**
 * @}
 */

/**
 * @defgroup LPS331AP_Exported_Macros         LPS331AP Exported Macros
 * @{
 */


/**
* @brief Read LPS331AP output register, and calculate the pressure in mbar.
* @retval float: pressure expressed in mbar.
*/
#define Lps331apReadPressure()          ((float)Lps331apReadRawPressure()/4096)


/**
* @brief Read LPS331AP output register, and calculate the temperature in C deg.
* @retval float: temperature expressed in C deg.
*/
#define Lps331apReadTemperature()       ((float)Lps331apReadRawTemperature()/480.0+42.5)


/** @defgroup LPS331AP_Communication        LPS331AP Communication
 * @{
 */
/**
 * @brief Mapping of Lps331apByteRead in @ref Lps331apBufferRead.
 */ 
#define Lps331apByteRead(pVal,cAddress)               Lps331apBufferRead(pVal,cAddress,1)
/**
 * @brief Mapping of Lps331apByteWrite in @ref Lps331apBufferWrite.
 */ 
#define Lps331apByteWrite(pVal,cAddress)              Lps331apBufferWrite(pVal,cAddress,1)


/**
 * @} 
 */

/**
 * @} 
 */  


/** @defgroup LPS331AP_Register_Mapping       LPS331AP Register Mapping
 * @{
 */

/**
 * @brief Reference pressure (LSB data)
 * \code
 * Read/write
 * Default value: 0x00
 * 7:0 REF7-ODR0: Lower part of the reference pressure that
 *      is sum to the sensor output pressure.
 * \endcode
 */
#define LPS_REF_P_XL_ADDR         0x08


/**
 * @brief Reference pressure (middle part)
 * \code
 * Read/write
 * Default value: 0x00
 * 7:0 REF15-ODR8: Middle part of the reference pressure that
 *      is sum to the sensor output pressure.
 * \endcode
 */
#define LPS_REF_P_L_ADDR          0x09


/**
 * @brief Reference pressure (MSB part)
 * \code
 * Read/write
 * Default value: 0x00
 * 7:0 REF15-ODR8: Higher part of the reference pressure that
 *      is sum to the sensor output pressure.
 * \endcode
 */
#define LPS_REF_P_H_ADDR          0x0A


/**
 * @brief Device identifier register.
 * \code
 * Read
 * Default value: 0xBB
 * 7:0 This read-only register contains the device identifier that, for LPS331AP, is set to BBh.
 * \endcode
 */
#define LPS_WHO_AM_I_ADDR                 0x0F


/**
 * @brief Pressure resolution Register
 * \code
 * Read/write
 * Default value: 0x7A
 * 7 RFU
 * 6:4 AVGT2-AVGT0: temperature internal average.
 *     AVGT2 | AVGT1 | AVGT0 | Nr. Internal Average
 *   ------------------------------------------------------
 *      0    |  0    |  0    |     1 
 *      0    |  0    |  1    |     2   
 *      0    |  1    |  0    |     4    
 *      0    |  1    |  1    |     8 
 *      1    |  0    |  0    |     16   
 *      1    |  0    |  1    |     32
 *      1    |  1    |  0    |     64
 *      1    |  1    |  1    |     128
 *
 * 3:0 AVGP3-AVGP0: pressure internal average.
 *     AVGP3 | AVGP2 | AVGP1 |  AVGP0 | Nr. Internal Average
 *   ------------------------------------------------------
 *      0    |  0    |  0    |   0    |      1
 *      0    |  0    |  0    |   1    |      2   
 *      0    |  0    |  1    |   0    |      4   
 *      0    |  0    |  1    |   1    |      8
 *      0    |  1    |  0    |   0    |      16   
 *      0    |  1    |  0    |   1    |      32   
 *      0    |  1    |  1    |   0    |      64
 *      0    |  1    |  1    |   1    |      128   
 *      1    |  0    |  0    |   0    |      256   
 *      1    |  0    |  0    |   1    |      384
 *      1    |  0    |  1    |   0    |      512   
 *
 * \endcode
 */
#define LPS_RES_CONF_ADDR                 0x10 


/**
 * @brief Pressure sensor control register 1
 * \code
 * Read/write
 * Default value: 0x00
 * 7 PD: power down control. 0 - disable; 1 - enable
 * 6:4 ODR2, ODR1, ODR0: output data rate selection.
 *     ODR2  | ODR1  | ODR0  | Pressure output data-rate(Hz)  | Temperature output data-rate(Hz)
 *   ----------------------------------------------------------------------------------
 *      0    |  0    |  0    |         one shot               |         one shot 
 *      0    |  0    |  1    |            1                   |            1 
 *      0    |  1    |  0    |            7                   |            1     
 *      0    |  1    |  1    |            12.5                |            1  
 *      1    |  0    |  0    |            25                  |            1    
 *      1    |  0    |  1    |            7                   |            7 
 *      1    |  1    |  0    |            12.5                |            12.5 
 *      1    |  1    |  1    |            25                  |            25
 *
 * 3 DIFF_EN: Interrupt circuit. 0 - disable; 1 - enable
 * 2 BDU: block data update. 0 - disable; 1 - enable
 * 1 DELTA_EN: delta pressure. 0 - disable; 1 - enable
 * 0 SIM: SPI Serial Interface Mode selection. 0 - disable; 1 - enable
 * \endcode
 */
#define LPS_CTRL_REG1_ADDR                    0x20


/**
 * @brief Pressure sensor control register 2
 * \code
 * Read/write
 * Default value: 0x00
 * 7 BOOT:  Reboot memory content. 0: normal mode; 1: reboot memory content
 * 6:3 Reserved.
 * 2 SWRESET: Software reset. 0: normal mode; 1: SW reset.
 * 1 AUTO_ZERO: Autozero enable. 0: normal mode; 1: autozero enable.
 * 0 ONE_SHOT: One shot enable. 0: waiting for start of conversion; 1: start for a new dataset
 * \endcode
 */
#define LPS_CTRL_REG2_ADDR                      0x21


/**
 * @brief Pressure sensor control register 3
 * \code
 * Read/write
 * Default value: 0x00
 * 7 INT_H_L: Interrupt active high, low.
 * 6 PP_OD: Push-pull/open drain selection on interrupt pads.
 * 5:3 INT2_S3, INT2_S2, INT2_S1: data signal on INT2 pad control bits.
 * 2:0 INT1_S3, INT1_S2, INT1_S1: data signal on INT1 pad control bits.
 *     INT1(2)_S3 | INT1(2)_S2  | INT1(2)_S1  | INT1(2) pin
 *   ------------------------------------------------------
 *        0       |     0       |      0      |     GND 
 *        0       |     0       |      1      |     Pressure high (P_high) 
 *        0       |     1       |      0      |     Pressure low (P_low)     
 *        0       |     1       |      1      |     P_low OR P_high 
 *        1       |     0       |      0      |     Data ready   
 *        1       |     0       |      1      |     Reserved
 *        1       |     1       |      0      |     Reserved
 *        1       |     1       |      1      |     Tri-state

 * \endcode
 */
#define LPS_CTRL_REG3_ADDR                    0x22 


/**
 * @brief Interrupt configuration Register
 * \code
 * Read/write
 * Default value: 0x00.
 * 7:3 Reserved.
 * 2 LIR: Latch Interrupt request into INT_SOURCE register. 0 - disable; 1 - enable
 * 1 PL_E: Enable interrupt generation on differential pressure low event. 0 - disable; 1 - enable
 * 0 PH_E: Enable interrupt generation on differential pressure high event. 0 - disable; 1 - enable
 * \endcode
 */
#define LPS_INT_CFG_REG_ADDR                  0x23 


/**
 * @brief Interrupt source Register
 * \code
 * Read
 * Default value: 0x00.
 * 7:3 0.
 * 2 IA: Interrupt Active.0: no interrupt has been generated; 1: one or more interrupt events have been generated.
 * 1 PL: Differential pressure Low. 0: no interrupt has been generated; 1: Low differential pressure event has occurred.
 * 0 PH: Differential pressure High. 0: no interrupt has been generated; 1: High differential pressure event has occurred.
 * \endcode
 */
#define LPS_INT_SOURCE_REG_ADDR               0x24 


/**
 * @brief Threshold pressure (LSB)
 * \code
 * Read
 * Default value: 0x00.
 * 7:0 THS7-THS0: Low part of threshold value for pressure interrupt
 * generation. The complete threshold value is given by THS_P_H & THS_P_L and is
 * expressed as unsigned number. P_ths(mbar)=(THS_P_H & THS_P_L)[dec]/16.
 * \endcode
 */
#define LPS_THS_P_LOW_REG_ADDR                0x25 


/**
 * @brief Threshold pressure (MSB)
 * \code
 * Read
 * Default value: 0x00.
 * 7:0 THS15-THS8: High part of threshold value for pressure interrupt
 * generation. The complete threshold value is given by THS_P_H & THS_P_L and is
 * expressed as unsigned number. P_ths(mbar)=(THS_P_H & THS_P_L)[dec]/16.
 * \endcode
 */
#define LPS_THS_P_HIGH_REG_ADDR              0x26 


/**
 * @brief  Status Register
 * \code
 * Read
 * Default value: 0x00
 * 7:6 0
 * 5 P_OR: Pressure data overrun. 0: no overrun has occurred; 1: new data for pressure has overwritten the previous one.
 * 4 T_OR: Temperature data overrun. 0: no overrun has occurred; 1: a new data for temperature has overwritten the previous one.
 * 3:2 0
 * 1 P_DA: Pressure data available. 0: new data for pressure is not yet available; 1: new data for pressure is available.
 * 0 T_DA: Temperature data available. 0: new data for temperature is not yet available; 1: new data for temperature is available.
 * \endcode
 */
#define LPS_STATUS_REG_ADDR                 0x27 


/**
 * @brief  Pressure data (LSB).
 * \code
 * Read
 * Default value: 0x00.
 * POUT7 - POUT0: Pressure data LSB (2's complement).     
 * Pressure output data: Pout(mbar)=(PRESS_OUT_H & PRESS_OUT_L &
 * PRESS_OUT_XL)[dec]/4096.
 * \endcode
 */
#define LPS_PRESS_POUT_XL_ADDR              0x28 


/**
 * @brief  Pressure data (Middle part).
 * \code
 * Read
 * Default value: 0x80.
 * POUT15 - POUT8: Pressure data middle part (2's complement).    
 * Pressure output data: Pout(mbar)=(PRESS_OUT_H & PRESS_OUT_L &
 * PRESS_OUT_XL)[dec]/4096.
 * \endcode
 */
#define LPS_PRESS_OUT_L_ADDR                0x29


/**
 * @brief  Pressure data (MSB).
 * \code
 * Read
 * Default value: 0x2F.
 * POUT23 - POUT16: Pressure data MSB (2's complement).
 * Pressure output data: Pout(mbar)=(PRESS_OUT_H & PRESS_OUT_L &
 * PRESS_OUT_XL)[dec]/4096.
 * \endcode
 */
#define LPS_PRESS_OUT_H_ADDR                0x2A


/**
 * @brief  Temperature data (LSB).
 * \code
 * Read
 * Default value: 0x00.
 * TOUT7 - TOUT0: temperature data LSB. 
 * T(degC) = 42.5 + (Temp_OUTH & TEMP_OUT_L)[dec]/480.
 * \endcode
 */
#define LPS_TEMP_OUT_L_ADDR                 0x2B


/**
 * @brief  Temperature data (MSB).
 * \code
 * Read
 * Default value: 0x00.
 * TOUT15 - TOUT8: temperature data MSB. 
 * T(degC) = 42.5 + (Temp_OUTH & TEMP_OUT_L)[dec]/480.
 * \endcode
 */
#define LPS_TEMP_OUT_H_ADDR                 0x2C 


/**
 * @brief Analog front end control
 * \code
 * Read/write
 * Default value: 0x00
 * 7:1 Reserved.
 * 0 SELMAIN: Current of operational amplifier selector.
 * -‘1’ always high current
 * -‘0’ high current during pressure acquisition and low current during temperature acquisition
 * \endcode
 */
#define LPS_AMP_CTRL_ADDR                   0x30 


/**
 * @brief  Pressure offset (LSB).
 * \code
 * Read
 * Default value: 0x00.
 * DELTA0 - DELTA7: Delta pressure register for One Point calibration.   
 * \endcode
 */
#define LPS_DELTA_PRESS_XL_ADDR             0x3C 


/**
 * @brief  Pressure offset (Middle part).
 * \code
 * Read
 * Default value: 0x80.
 * DELTA15-DELTA8: Delta pressure register for One Point calibration.  
 * \endcode
 */
#define LPS_DELTA_PRESS_L_ADDR              0x3D


/**
 * @brief  Pressure offset (MSB).
 * \code
 * Read
 * Default value: 0x2F.
 * DELTA23-DELTA16: Delta pressure register for One Point calibration.
 * \endcode
 */
#define LPS_DELTA_PRESS_H_ADDR              0x3E

/**
 * @}
 */



/** @defgroup LPS331AP_Exported_Functions           LPS331AP Exported Functions
 * @{
 */

void Lps331apConfig(LPS331Init* pxLPS331InitStruct);
void Lps331apGetInfo(LPS331Init* pxLPS331InitStruct);
void Lps331apLowPowerMode(LPSFunctionalState xFunctionalState);
void Lps331apSetPressureResolution(LPSPressureResolution xPressureResolution);
void Lps331apSetTemperatureResolution(LPSTemperatureResolution xTemperatureResolution);
void Lps331apSetDataRate(LPSOutputDataRate xDataRate);
LPSOutputDataRate Lps331apGetDataRate(void);
void Lps331apRebootCmd(void);
void Lps331apEnterShutdownCmd(void);
void Lps331apExitShutdownCmd(void);
int32_t Lps331apReadRawPressure(void);
int16_t Lps331apReadRawTemperature(void);
void Lps331apIrqInit(LPSIrqInit* pxLPSIrqInit);
void Lps331apOneShot(void);
LPS331DataStatus Lps331apGetDataStatus(void);
void Lps331apSetThreshold(float fThreshold);
float Lps331apGetThreshold(void);

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

/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/

