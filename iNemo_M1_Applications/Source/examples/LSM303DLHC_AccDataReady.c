/**
 * @file    LSM303DLHC_AccDataReady.c
 * @author  ART Team IMS-Systems Lab
 * @version V1.0.0
 * @date    May 30, 2012
 * @brief   Example of data acquisition from accelerometer/magnetometer on DATA READY.
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
#include "LSM303DLHC.h"


#define USE_VCOM
#define USE_DFU

#ifdef USE_VCOM
#include "STM32F1_VC_General.h"
#endif

/** @addtogroup iNemo_M1_Examples                           INEMO-M1 Examples
 * @{
 */

/** @addtogroup iNemo_LSM303DLHC_DataReady                    iNemo LSM303DLHC DataReady
 * @brief  This code explain how to configure and manage the LSM303DLH acc/mag sensor for data
 *         reading on IRQ.
 * @{
 */


/**
 * @defgroup LSM303DLHC_DataReady_Private_Variables               LSM303DLHC DataReady Private Variables
 * @{
 */

/**
 * @brief Accelerometer sensor init structure
 */
LSMAccInit LSMAccInitStructure;


/**
* @brief Accelerometer high pass filter init structure
*/
LSMAccFilterInit LSMAccFilterInitStructure;


/**
* @brief Magnetometer sensor init structure
*/
LSMMagInit LSMMagInitStructure;


/**
 * @brief Acceletometer DataReady flag
 */
FlagStatus xAccDataReady = RESET;


/**
 * @brief Acceleration and Magnetic field values
 */
float fAccXYZ[3],fMagXYZ[3];

/**
 * @brief Temperature value
 */
float fTemperature;

/**
 * @brief Accelerometer status
 */
LSMADataStatus xAccStatus;


/**
 * @brief Magnetometer status
 */
LSMMDataStatus xMagStatus;


/**
 *@}
 */


/**
 * @defgroup LSM303DLHC_DataReady_Private_Functions                                LSM303DLHC DataReady Private Functions
 * @{
 */

/**
 * @brief  This configures the Exit line associated with LSM303DLH data ready line.
 * @param  None
 * @retval None
 */
void ExtiConfiguration(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

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
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}


/**
 * @brief  This function handles External interrupt request (associated with LSM303DLHC data ready line).
 * @param  None
 * @retval None
 */
void EXTI9_5_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line8) != RESET)
  {
    /* set the DataReady flag */
    xAccDataReady = SET;

    /* Clear the pending bit */
    EXTI_ClearITPendingBit(EXTI_Line8);
  }

}



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
  Lsm303dlhcI2CInit();
  
  for(uint32_t i=0; i<0xFFFFFF;i++)
  {
    if(i%0xFFFFF==0)
    {
      Stm32f1VCPrintf("waiting... %d \r",i/0xFFFFF);
      Stm32f1VCSendData();
    }
  }
  
  /* Fill the accelerometer structure */
  LSMAccInitStructure.xPowerMode = LSM_NORMAL_MODE;
  LSMAccInitStructure.xOutputDataRate = LSM_ODR_50_HZ;
  LSMAccInitStructure.xEnabledAxes= LSM_ALL_AXES_EN;
  LSMAccInitStructure.xFullScale = LSM_FS_2G;
  LSMAccInitStructure.xDataUpdate = LSM_CONTINUOS_UPDATE;
  LSMAccInitStructure.xEndianness=LSM_BIG_ENDIAN;
  LSMAccInitStructure.xHighResolution=LSM_ENABLE;

  /* Fill the accelerometer LPF structure */
  LSMAccFilterInitStructure.xHPF=LSM_DISABLE;
  LSMAccFilterInitStructure.xHPF_Mode=LSM_HPFM_NORMAL;
  LSMAccFilterInitStructure.cHPFReference=0x00;
  LSMAccFilterInitStructure.xHPFCutOff=LSM_HPCF_16;
  LSMAccFilterInitStructure.xHPFClick=LSM_DISABLE;
  LSMAccFilterInitStructure.xHPFAOI2=LSM_DISABLE;
  LSMAccFilterInitStructure.xHPFAOI1=LSM_DISABLE;


  /* Fill the magnetometer structure */
  LSMMagInitStructure.xOutputDataRate = LSM_ODR_30_HZ;
  LSMMagInitStructure.xFullScale = LSM_FS_1_3_GA;
  LSMMagInitStructure.xWorkingMode = LSM_CONTINUOS_CONVERSION;
  LSMMagInitStructure.xTemperatureSensor = LSM_ENABLE ;

  /* Configure the MCU exti */
  ExtiConfiguration();

  /* Configure the sensor IRQ */
  Lsm303dlhcAccIrq1Config(LSM_I1_DRDY1,LSM_ENABLE);

  /* Configure the accelerometer main parameters */
  Lsm303dlhcAccConfig(&LSMAccInitStructure);

  /* Configure the accelerometer LPF main parameters */
  Lsm303dlhcAccFilterConfig(&LSMAccFilterInitStructure);

  /* Configure the magnetometer main parameters */
  Lsm303dlhcMagConfig(&LSMMagInitStructure);


  while(1)
  {
    /* Wait for data ready (set by the proper ISR) */
    while(!xAccDataReady);
    xAccDataReady=RESET;

    /* Read the data status */
    xAccStatus =Lsm303dlhcAccGetDataStatus();
    xMagStatus = Lsm303dlhcMagGetDataStatus();


    /* Read data */
    Lsm303dlhcAccReadAcc(fAccXYZ);
    Lsm303dlhcMagReadMag(fMagXYZ);
    fTemperature=Lsm303dlhcMagReadTemp();

#ifdef USE_VCOM
    /* Print data via Virtual Com */
    Stm32f1VCPrintf("ACCStatus: ( %d,%d,%d,%d,%d,%d - %d,%d )\n\rACC: X=%f, Y=%f, Z=%f\n\rMAG: X=%f, Y=%f, Z=%f\n\rTemperature=%f\n\n\r", xAccStatus.Z_Or,xAccStatus.Y_Or,xAccStatus.X_Or,xAccStatus.Z_Da,xAccStatus.Y_Da,xAccStatus.X_Da,xMagStatus.xDataReady,xMagStatus.xDataLock,
                  fAccXYZ[0], fAccXYZ[1], fAccXYZ[2], fMagXYZ[0], fMagXYZ[1], fMagXYZ[2], fTemperature);

    Stm32f1VCSendData();
#endif
   
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
