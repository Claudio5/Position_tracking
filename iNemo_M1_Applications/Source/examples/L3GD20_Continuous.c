/**
 * @file    L3GD20_Continuous.c
 * @author  ART Team IMS-Systems Lab
 * @version V1.0.0
 * @date    May 30, 2012
 * @brief   Example of data acquisition from gyroscope in continuous mode (w/o IRQ).
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "L3Gx.h"

#define USE_VCOM
#define USE_DFU

#ifdef USE_VCOM
#include "STM32F1_VC_General.h"
#endif


/** @addtogroup iNemo_M1_Examples                           INEMO-M1 Examples
 * @{
 */

/** @addtogroup iNemo_L3GD20_Continuous                    iNemo L3GD20 Continuous
 * @brief  This code explain how to configure and manage the L3GD20 reading data
 *          in continuous mode (w/o IRQ).
 * @{
 */


/**
 * @defgroup L3GD20_Continuous_Private_Variables               L3GD20 Continuous Private Variables
 * @{
 */


/**
 * @brief Gyroscopic sensor init structure
 */
L3GInit L3GInitStructure;


/**
 * @brief Gyroscope data
 */
float fGyroXYZ[3];


/**
 *@}
 */


/**
 * @defgroup L3GD20_Continuous_Private_Functions                                L3GD20 Continuous Private Functions
 * @{
 */

int main(void)
{
  /* At this stage the microcontroller clock setting is already configured,
  this is done through SystemInit() function which is called from startup
  file (startup_stm32f10x_xx.s) before to branch to application main.
  To reconfigure the default setting of SystemInit() function, refer to
  system_stm32f10x.c file
  */
#ifdef USE_DFU
	/* Set the Vector Table base address at 0x08003000 */
	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x3000);
#endif

  for(uint32_t i=0; i<0xFFFFF;i++);

#ifdef USE_VCOM
  /* Initialize Virtual Com */
  Stm32f1VCInit();
#endif

  /* Initialize the MCU digital interface to communicate with the sensor */
  L3gd20CommInit();

  /* Fill the gyro structure */
  L3GInitStructure.xPowerMode = L3G_NORMAL_SLEEP_MODE;
  L3GInitStructure.xOutputDataRate = L3G_ODR_190_HZ_CUTOFF_12_5;
  L3GInitStructure.xEnabledAxes = L3G_ALL_AXES_EN;
  L3GInitStructure.xFullScale = L3G_FS_500_DPS;
  L3GInitStructure.xDataUpdate = L3G_CONTINUOS_UPDATE;
  L3GInitStructure.xEndianness = L3G_BIG_ENDIAN;

  /* Configure the gyro main parameters */
  L3gd20Config(&L3GInitStructure);

  /* Wait some seconds in order to ensure the user opens the VCOM terminal */
  for(uint32_t i=0;i<0x1FFFFFF;i++);

  while(1)
  {
    /* Read gyro data */
    L3gd20ReadAngRate(fGyroXYZ);

#ifdef USE_VCOM
    /* Print it */
    Stm32f1VCPrintf("%f %f %f\n\r", fGyroXYZ[0],fGyroXYZ[1],fGyroXYZ[2]);
    Stm32f1VCSendData();
#endif

    /* Wait some seconds */
    for(uint32_t i=0;i<0xFFFFF;i++);

  }
}



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}

#endif
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
