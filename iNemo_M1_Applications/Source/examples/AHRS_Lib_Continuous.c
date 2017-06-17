/**
 * @file    AHRS_Lib_Continuous.c
 * @author  ART Team IMS-Systems Lab
 * @version V1.0.0
 * @date    May 30, 2012
 * @brief   Example of AHRS with Euler Angles and quaternions output in continuous mode.
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
#include "L3Gx.h"
#include "iNEMO_AHRS.h"
#include "iNemo_Led.h"


#define USE_VCOM
#define USE_DFU

#ifdef USE_VCOM
#include "STM32F1_VC_General.h"
#endif


/** @addtogroup iNemo_M1_Examples                           iNemo M1 Examples
 * @{
 */

/** @addtogroup iNemo_AHRS_Continuous                    iNemo AHRS Continuous
 * @brief  This code explain how to configure and use the AHRS algorithm.
 * @note   A heap size of at least 0x1000 is required for this application. The IAR
 *          linker file stm32f10x_flash_offset_ExpHeap.icf configures a heap with this dimension.
 * @{
 */


/**
 * @defgroup AHRS_Continuous_Private_Variables               AHRS Continuous Private Variables
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
 * @brief Gyroscopic sensor init structure
 */
L3GInit L3GInitStructure;


/**
 * @brief Acceleration and Magnetic field values
 */
float fAccXYZ[3], fMagXYZ[3];


/**
 * @brief Gyroscope data
 */
float fGyroXYZ[3];

/**
 * @brief AHRS filter input data structure
 */
iNEMO_SENSORDATA xSensorData;

iNEMO_EULER_ANGLES xEulerAngles={0};

iNEMO_QUAT  xQuat={0};

FlagStatus xTim2Raised = RESET;

uint8_t iter=0;
/**
 *@}
 */


/**
 * @defgroup AHRS_Continuous_Private_Functions                                AHRS Continuous Private Functions
 * @{
 */

#define abs(a) ((a)>0?(a):-(a))

/**
 * \brief  Compute two integer value a and b such that n = a * b. It used to
 *                  setup the timer to generate an IRQ whit a specified frequency n.
 * \param  n : the specified frequency
 * \param  a : prescaler factor
 * \param  b : period factor
 * \retval None
*/

void prvFindFactors(u32 n, uint16_t *a, uint16_t *b)
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
 * @brief Configures the timer 2
 */
void iNemoTimerConfig(void)
{
  unsigned short a;
  unsigned short b;
  unsigned long n;
  /* This value is the frequency interrupts in Hz */
  unsigned char frequency = 50;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable timer clocks */
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );

  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

  TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );

  /* Time base configuration for timer 2 - which generates the interrupts. */
  n = SystemCoreClock/frequency;

  prvFindFactors( n, &a, &b );
  TIM_TimeBaseStructure.TIM_Period = b - 1;
  TIM_TimeBaseStructure.TIM_Prescaler = a - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure );
  TIM_ARRPreloadConfig( TIM2, ENABLE );

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( &NVIC_InitStructure );

}


/**
 * @brief Start and stop the timer used by the iNemo Data Task.
 * @param command start the timer if ENABLE, stop the timer if DISABLE.
 * @retval None.
 */
void Enable_Timer(FunctionalState command) {
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_ITConfig( TIM2, TIM_IT_Update, command );
	TIM_Cmd(TIM2, command);
}


/**
 * @brief This function handles TIM2 global interrupt request by resuming the
 * iNemoData task.
 */
void TIM2_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM2, TIM_IT_Update))
  {
    xTim2Raised=SET;

    /* Clear the IRQ bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
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

  float temp[2];

  for(uint32_t i=0; i<0xFFFFF;i++);

  /* Initialize the iNemo LED */
  iNEMO_Led_Init(LED1);

#ifdef USE_VCOM
  /* Initialize Virtual Com */
  Stm32f1VCInit();
#endif

  iNemoTimerConfig();
  
  /* Filter references for Acceleration and Magnetic field */
  xSensorData.m_fAccRef[0]=0;
  xSensorData.m_fAccRef[1]=0;
  xSensorData.m_fAccRef[2]=-9.81f;
      
  xSensorData.m_fMagRef[0]=0.37f;
  xSensorData.m_fMagRef[1]=0;
  xSensorData.m_fMagRef[2]=-0.25f;
  
  xSensorData.m_fDeltaTime=0.02f;
  
  xSensorData.m_fVarAcc=5.346e-6;
  xSensorData.m_fVarMag=5.346e-6;


  iNEMO_AHRS_Init(&xSensorData, &xEulerAngles, &xQuat);

  /* Initialize the MCU digital interface to communicate with the sensor */
  Lsm303dlhcI2CInit();
  /* Initialize the MCU digital interface to communicate with the sensor */
  L3gd20CommInit();


  /* Fill the accelerometer structure */
  LSMAccInitStructure.xPowerMode = LSM_NORMAL_MODE;
  LSMAccInitStructure.xOutputDataRate = LSM_ODR_400_HZ;
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
  LSMMagInitStructure.xFullScale = LSM_FS_1_9_GA;
  LSMMagInitStructure.xWorkingMode = LSM_CONTINUOS_CONVERSION;
  LSMMagInitStructure.xTemperatureSensor = LSM_ENABLE ;


  /* Configure the accelerometer main parameters */
  Lsm303dlhcAccConfig(&LSMAccInitStructure);

  /* Configure the accelerometer LPF main parameters */
  Lsm303dlhcAccFilterConfig(&LSMAccFilterInitStructure);

  /* Configure the magnetometer main parameters */
  Lsm303dlhcMagConfig(&LSMMagInitStructure);

  /* Fill the gyro structure */
  L3GInitStructure.xPowerMode = L3G_NORMAL_SLEEP_MODE;
  L3GInitStructure.xOutputDataRate = L3G_ODR_190_HZ_CUTOFF_12_5;
  L3GInitStructure.xEnabledAxes = L3G_ALL_AXES_EN;
  L3GInitStructure.xFullScale = L3G_FS_500_DPS;
  L3GInitStructure.xDataUpdate = L3G_BLOCK_UPDATE;
  L3GInitStructure.xEndianness = L3G_BIG_ENDIAN;

  /* Configure the gyro main parameters */
  L3gd20Config(&L3GInitStructure);


  /* Wait some seconds in order to ensure the user opens the VCOM terminal */
  for(uint32_t i=0;i<0x1FFFFFF;i++);

  Enable_Timer(ENABLE);

  while(1)
  {

    while(!xTim2Raised);
    xTim2Raised = RESET;

    /* Read data */
    Lsm303dlhcAccReadAcc(fAccXYZ);
    Lsm303dlhcMagReadMag(fMagXYZ);
    L3gd20ReadAngRate(fGyroXYZ);

    temp[0]= -fGyroXYZ[1];
    temp[1] = fGyroXYZ[0];
    fGyroXYZ[0] = temp[0];
    fGyroXYZ[1] = temp[1];


    {
      xSensorData.m_fAcc[0]=fAccXYZ[0]*9.8f/1000.0f;
      xSensorData.m_fMag[0]=fMagXYZ[0]/1000.0;
      xSensorData.m_fGyro[0]=fGyroXYZ[0]*3.141592f/180.0f;


      xSensorData.m_fAcc[1]=-fAccXYZ[1]*9.8f/1000.0f;
      xSensorData.m_fMag[1]=-fMagXYZ[2]/1000.0;
      xSensorData.m_fGyro[1]=-fGyroXYZ[1]*3.141592f/180.0f;

      xSensorData.m_fAcc[2]=-fAccXYZ[2]*9.8f/1000.0f;
      xSensorData.m_fMag[2]=-fMagXYZ[1]/1000.0;
      xSensorData.m_fGyro[2]=-fGyroXYZ[2]*3.141592f/180.0f;
      
    }

    iNEMO_AHRS_Update(&xSensorData, &xEulerAngles, &xQuat);

#ifdef USE_VCOM
    if(iter++ == 25)
    {
      /* Print data via Virtual Com */
      Stm32f1VCPrintf("ACC(mg): X=%f, Y=%f, Z=%f\n\r", fAccXYZ[0], fAccXYZ[1], fAccXYZ[2]);
      Stm32f1VCPrintf("GYRO(dps): X=%.3f, Y=%.3f, Z=%.3f\n\r", fGyroXYZ[0], fGyroXYZ[1], fGyroXYZ[2]);
      Stm32f1VCPrintf("MAG(mgauss): X=%.3f, Y=%.3f, Z=%.3f\n\n\r", fMagXYZ[0], fMagXYZ[1], fMagXYZ[2]);
      Stm32f1VCPrintf("Attitude(deg): R=%.3f, P=%.3f, Y=%.3f\n\n\r", xEulerAngles.m_fRoll* 180.0f / 3.141592f, xEulerAngles.m_fPitch * 180.0f / 3.141592f, xEulerAngles.m_fYaw * 180.0f / 3.141592f);
      Stm32f1VCSendData();
      iter=0;
    }
#endif



    /* Toggle the iNemo led */
    iNEMO_Led_Toggle(LED1);
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
