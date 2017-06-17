/**
 * @file    LPS331AP_DataReady.c
 * @author  ART Team IMS-Systems Lab
 * @version V1.0.0
 * @date    May 30, 2012
 * @brief   Example of data acquisition from pressure sensor.
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
#include "LPS331AP.h"

#include "iNemo_Led.h"

#define USE_VCOM
#define USE_DFU

#ifdef USE_VCOM
#include "STM32F1_VC_General.h"
#endif

/** @addtogroup iNemo_M1_Examples                           INEMO-M1 Examples
 * @{
 */

/** @addtogroup iNemo_LPS331AP_DataReady                    iNemo LPS331AP DataReady
 * @brief  This code explain how to configure and manage the LPS331AP pressure sensor for data
 *         reading on IRQ.
 * @{
 */


/**
 * @defgroup LPS331AP_DataReady_Private_Variables               LPS331AP DataReady Private Variables
 * @{
 */

/**
 * @brief Pressure sensor init structure
 */
LPS331Init xLps331apInit;


/**
 * @brief Pressure sensor IRQ configuration structure
 */
LPSIrqInit xLps331apIrqInit;


/**
 * @brief Pressure and temperature values
 */
float fPressure,fTemperature;


/**
 * @brief IRQ data ready flag
 */
FlagStatus xDataReady=RESET;

/**
 *@}
 */


/**
 * @defgroup LPS331AP_DataReady_Private_Functions                                LPS331AP DataReady Private Functions
 * @{
 */

/**
 * @brief  This configures the Exit line associated with LPS331AP data ready line.
 * @param  None
 * @retval None
 */
void ExtiConfiguration(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

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
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;

  EXTI_Init(&EXTI_InitStructure);

  EXTI_ClearITPendingBit(EXTI_Line0);
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}


/**
 * @brief  This function handles External interrupt request (associated with LPS331AP data ready line).
 * @param  None
 * @retval None
 */
void EXTI0_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
    /* set the DataReady flag */
    xDataReady = SET;

    /* Clear the pending bit */
    EXTI_ClearITPendingBit(EXTI_Line0);
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
  LPS331DataStatus cStatus;

#ifdef USE_DFU
	/* Set the Vector Table base address at 0x08003000 */
	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x3000);
#endif

  for(uint32_t i=0; i<0xFFFFF;i++);

  /* Initialize the iNemo LED */
  iNEMO_Led_Init(LED1);

#ifdef USE_VCOM
  /* Initialize Virtual Com */
  Stm32f1VCInit();
#endif

  /* Initialize the MCU digital interface to communicate with the sensor */
  Lps331apCommInit();

  /* Fill the sensor structure */
  xLps331apInit.xOutputDataRate=ODR_P_7HZ_T_1HZ;
  xLps331apInit.xPresRes=LPS_PRESS_AVG_512;
  xLps331apInit.xTempRes=LPS_TEMP_AVG_1;
  xLps331apInit.xPressureAutoZero=LPS_DISABLE;
  xLps331apInit.fPressureRef=0;
  xLps331apInit.xBDU=LPS_DISABLE;

  /* Configure the sensor main parameters */
  Lps331apConfig(&xLps331apInit);

  /* Fill the sensor IRQ structure */
  xLps331apIrqInit.xIrqActiveLow = LPS_DISABLE;
  xLps331apIrqInit.xOutType = LPS_PP;
  xLps331apIrqInit.xInt1List = LPS_DATA_READY;
  xLps331apIrqInit.xInt2List = LPS_P_LOW_HIGH;
  xLps331apIrqInit.fPressThr = 1006.2;
  xLps331apIrqInit.xIrqPressLow=LPS_ENABLE;
  xLps331apIrqInit.xIrqPressHigh=LPS_ENABLE;

  /* Configure the MCU exti */
  ExtiConfiguration();

  /* Configure the sensor IRQ */
  Lps331apIrqInit(&xLps331apIrqInit);


  while(1)
  {
    /* Wait for data ready (set by the proper ISR) */
    while(!xDataReady);
    xDataReady=RESET;

    /* Read the data status */
    cStatus = Lps331apGetDataStatus();

    /* Read pressure and temperature */
    fPressure = Lps331apReadPressure();
    fTemperature = Lps331apReadTemperature();


#ifdef USE_VCOM
    /* Print data via Virtual Com */
    Stm32f1VCPrintf("\n\rstatus reg: (OR) %d - %d -(DA) %d - %d\n\rpressure: %f mbar\n\rtemperature: %f deg\n\r",cStatus.cTempDataOverrun,cStatus.cPressDataOverrun,cStatus.cTempDataAvailable,cStatus.cPressDataAvailable,fPressure, fTemperature);
    Stm32f1VCSendData();
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
