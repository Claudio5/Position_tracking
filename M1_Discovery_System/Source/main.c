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
#include <math.h>

#include "stm32f10x.h"
#include "LSM303DLHC.h"
#include "L3Gx.h"
#include "iNEMO_AHRS.h"
#include "iNemoLed.h"
#include "iNEMO_Button.h"
#include "position_get.h"
#include "iNemo.h"


#define USE_VCOM
#define USE_DFU

#ifdef USE_VCOM
#include "STM32F1_VC_General.h"
#endif


/*
 * This code realises the position calculation by integrating numericall the acceleration
 * */

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

Data data;

/**
 * @brief AHRS filter input data structure
 */
iNEMO_SENSORDATA xSensorData;

iNEMO_EULER_ANGLES xEulerAngles={0};

iNEMO_QUAT  xQuat={0};

FlagStatus xTim2Raised = RESET;

uint32_t iter=0;
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

void initSensorData()
{
	xSensorData.m_nCount=0;  /*!< It is used to perform correction at different frequencies */
	xSensorData.m_fDeltaTime=0;   /*!< It should trace the time difference */

	for(int i=0;i<3;i++)
	{
		xSensorData.m_fAcc[i]=0;		/*!< The Acceleration Measurement */
		xSensorData.m_fGyro[i]=0;		/*!< The Gyroscope Measurement */
		xSensorData.m_fMag[i]=0;
		xSensorData.m_fAccRef[i]=0;           /*!< The gravitational vector refer */
		xSensorData.m_fMagRef[i]=0;		/*!< The Magnitude Measurement */
	}

	for(int i=0;i<9;i++)
	{
		xSensorData.m_fScaleFactor[i]=0;      /*!< The Scale Factor Vector for each Measurement */
		xSensorData.m_fOffset[i]=0;		/*!< The Offset Vector for each Measurement */
	}
	           /*!< The magnetic vector refer */

	xSensorData.m_fVarAcc=0;              /*!< The accelerometer variance */
	xSensorData.m_fVarMag=0;              /*!< The magnetometer variance */
	xSensorData.m_fVarGyro=0;             /*!< The gyroscope variance */

}

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

    // Clear the IRQ bit
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }


}

/*
 * Function that rotates our acceleration vector
 * */
void rotateEuler(float out[3],float a[3], float angle[2])
{

	out[0]= ((float)cos((double)angle[1]))*a[0] - ((float)sin((double)angle[1]))*a[2];
	out[1]= (float)sin((double)angle[1])*(float)sin((double)angle[0])*a[0]+(float)cos((double)angle[0])*a[1]+ (float)cos((double)angle[1])*(float)sin((double)angle[0])*a[2];
	out[2]= (float)sin((double)angle[1])*(float)cos((double)angle[0])*a[0]-(float)sin((double)angle[0])*a[1]+ (float)cos((double)angle[1])*(float)cos((double)angle[0])*a[2];

}

/*
 * Check if the board is moving
 * */
int check_stationarity(float accX,float accY, float accZ)
{
	int stat;
	stat = stationary(accX,accY,accZ);

	return stat;
}

/*
 * Integrates numerically the acceleration
 * */
void calculating_velocity()
{

	data.velX = calculate_velocityX(data.velX,data.accX);
	data.velY = calculate_velocityY(data.velY,data.accY);
	data.velZ = calculate_velocityZ(data.velZ,data.accZ);

	if(data.stationary)
	{
		data.velX=0;
		data.velY=0;
		data.velZ=0;
	}
}
/*
 * Integrates numerically the acceleration
 * */
void calculating_position()
{

	data.posX = calculate_position(data.posX,data.velX);
	data.posY = calculate_position(data.posY,data.velY);
	data.posZ = calculate_position(data.posZ,data.velZ);

}

void getting_position()
{

	calculating_velocity();
	calculating_position();

}

void init_data()
{
	data.accX = 0;
	data.accY = 0;
	data.accZ = 0;

	data.gyrX = 0;
	data.gyrY = 0;
	data.gyrZ = 0;

	data.velX = 0;
	data.velY = 0;
	data.velZ = 0;

	data.driftVelX = 0;
	data.driftVelY = 0;
	data.driftVelZ = 0;

	data.posX = 0;
	data.posY = 0;
	data.posZ = 0;

	data.posX_filt=0;
	data.posY_filt=0;
	data.posZ_filt=0;

	data.magX = 0;
	data.magY = 0;
	data.magZ = 0;

	data.velX_prev = 0;
	data.velY_prev = 0;
	data.velZ_prev = 0;

	data.stationary = 0;

	data.q0 = 1;
	data.q1 = 0;
	data.q2 = 0;
	data.q3 = 0;

}



int main(void)
{

#ifdef USE_DFU
	/* Set the Vector Table base address at 0x08003000 */
	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x3000);
#endif

  float temp[2];

  //for(uint32_t i=0; i<0xFFFFF;i++);

  /* Initialize the iNemo LED */
  iNEMO_Led_Init(LED1);

#ifdef USE_VCOM
  /* Initialize Virtual Com */
  Stm32f1VCInit();
#endif

#ifdef USE_VCOM
  for(uint32_t i=0; i<0xFFFFF;i++)
 {
	 if(i%0xFFFF==0)
    {
      Stm32f1VCPrintf("waiting... \n\r");
      Stm32f1VCSendData();
    }
  }
#endif

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

  //Enable_Timer(ENABLE);

  float acc_temp[3];
  float out[3];
  float euler_ang[2];
  int start = 0;
  int end = 0;

  init_data();
  initSensorData();
  iNEMO_Button_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
  iNEMO_Led_Init(LED1);

  while(1)
  {
	  // wait for the user to press the button
	 if((!iNEMO_Button_Get_State(BUTTON_KEY)) && start==0)
	{
		start = 1;
		while(!iNEMO_Button_Get_State(BUTTON_KEY));
		end = 0;
		iter = 0;
	}
	else if(((!iNEMO_Button_Get_State(BUTTON_KEY))&& start==1)|| end)
	{

		init_data();

		start = 0;
		end = 0;

		while(!iNEMO_Button_Get_State(BUTTON_KEY));
	}

	 if(start)
		 iNEMO_Led_On(LED1);
	 else
		 iNEMO_Led_Off(LED1);


   /*while(!xTim2Raised);
    xTim2Raised = RESET;*/

	if(start)
	{

		for(uint32_t i=0; i<0xFFFFFF;i++);

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



		acc_temp[0] = fAccXYZ[0];
		acc_temp[1] = fAccXYZ[1];
		acc_temp[2] = fAccXYZ[2];

		data.gyrX = fGyroXYZ[0];
		data.gyrY = fGyroXYZ[1];
		data.gyrZ = fGyroXYZ[2];

		data.magX = fMagXYZ[0];
		data.magY = fMagXYZ[1];
		data.magZ = fMagXYZ[2];

		euler_ang[0]= -xEulerAngles.m_fRoll;
		euler_ang[1] = xEulerAngles.m_fPitch;


		if(euler_ang[1]*180.0f/3.141592f >= -2.5 && euler_ang[1]*180.0f/3.141592f <= 2.5)
			euler_ang[1] = 0;
		if(euler_ang[0]*180.0f/3.141592f >= -2.5 && euler_ang[0]*180.0f/3.141592f <= 2.5)
					euler_ang[0] = 0;

		rotateEuler(out,acc_temp,euler_ang);

		data.accX = out[0]/1000*9.81;
		data.accY = out[1]/1000*9.81;
		data.accZ = out[2]/1000*9.81-9.81;

		data.stationary=check_stationarity(data.accX, data.accY, data.accZ);

		getting_position();

	#ifdef USE_VCOM

		/* Print data via Virtual Com */
		//Stm32f1VCPrintf("ACC(mg): X=%f, Y=%f, Z=%f\n\r", fAccXYZ[0], fAccXYZ[1], fAccXYZ[2]);
		//Stm32f1VCPrintf("GYRO(dps): X=%.3f, Y=%.3f, Z=%.3f\n\r", fGyroXYZ[0], fGyroXYZ[1], fGyroXYZ[2]);
		// Stm32f1VCPrintf("MAG(mgauss): X=%.3f, Y=%.3f, Z=%.3f\n\n\r", fMagXYZ[0], fMagXYZ[1], fMagXYZ[2]);
		// Stm32f1VCPrintf("Attitude(deg): R=%.3f, P=%.3f, Y=%.3f\n\n\r", xEulerAngles.m_fRoll* 180.0f / 3.141592f, xEulerAngles.m_fPitch * 180.0f / 3.141592f, xEulerAngles.m_fYaw * 180.0f / 3.141592f);
		Stm32f1VCPrintf("Attitude(deg): R=%d \n\r", (int)(1000*xEulerAngles.m_fRoll* 180.0f / 3.141592f));
		Stm32f1VCPrintf("Attitude(deg): P=%d \n\r", (int)(1000*xEulerAngles.m_fPitch* 180.0f / 3.141592f));
		Stm32f1VCPrintf("Attitude(deg): Y=%d \n\r", (int)(1000*xEulerAngles.m_fYaw* 180.0f / 3.141592f));
		Stm32f1VCSendData();
		Stm32f1VCPrintf("ACC_PREV(mg): X=%d, Y=%d, Z=%d\n\r", (int)fAccXYZ[0],(int) fAccXYZ[1],(int) fAccXYZ[2]);
		Stm32f1VCSendData();
		Stm32f1VCPrintf("ACC(mg): X=%d, Y=%d, Z=%d\n\r", (int)out[0],(int) out[1],(int) out[2]);
		Stm32f1VCSendData();
		Stm32f1VCPrintf("POS(mm): X=%d, Y=%d, Z=%d\n\r", (int)(data.posX*1000),(int) (data.posY*1000),(int) (data.posZ*1000));
		Stm32f1VCPrintf("------------------------------------------------------------------\n\r");
		Stm32f1VCSendData();
	#endif
		if(iter++ == 500)
		{
			end = 1;
			iter=0;
		}


		/* Toggle the iNemo led */
		//iNEMO_Led_Toggle(LED1);
	  }
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
