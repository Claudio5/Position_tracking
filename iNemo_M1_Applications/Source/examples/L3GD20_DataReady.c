/**
 * @file    L3GD20_DataReady.c
 * @author  ART Team IMS-Systems Lab
 * @version V1.0.0
 * @date    May 30, 2012
 * @brief   Example of data acquisition from gyroscope.
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

/** @addtogroup iNemo_L3GD20_DataReady                    iNemo L3GD20 DataReady
 * @brief  This code explain how to configure and manage the L3GD20 gyro sensor for data
 *         reading on IRQ.
 * @{
 */


/**
 * @defgroup L3GD20_DataReady_Private_Variables               L3GD20 DataReady Private Variables
 * @{
 */


FlagStatus xDataReady=RESET;

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
 * @defgroup L3GD20_DataReady_Private_Functions                                L3GD20 DataReady Private Functions
 * @{
 */

/**
 * @brief  This configures the Exit line associated with L3GD20 data ready line.
 * @param  None
 * @retval None
 */
void ExtiConfiguration(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource6);

  EXTI_InitStructure.EXTI_Line = EXTI_Line6;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;

  EXTI_Init(&EXTI_InitStructure);

  EXTI_ClearITPendingBit(EXTI_Line6);
  
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

void PwmConfig(void)
{

  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* GPIOA clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 665;
  TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 24000000) - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC2Init(TIM2, &TIM_OCInitStructure);

  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);
}


void EXTI9_5_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line6) != RESET)
  {
    /* set the DataReady flag */
    xDataReady = SET;

    /* Clear the pending bit */
    EXTI_ClearITPendingBit(EXTI_Line6);
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
  L3gd20CommInit();

  /* Fill the gyro structure */
  L3GInitStructure.xPowerMode = L3G_NORMAL_SLEEP_MODE;
  L3GInitStructure.xOutputDataRate = L3G_ODR_95_HZ_CUTOFF_12_5;
  L3GInitStructure.xEnabledAxes = L3G_ALL_AXES_EN;
  L3GInitStructure.xFullScale = L3G_FS_500_DPS;
  L3GInitStructure.xDataUpdate = L3G_CONTINUOS_UPDATE;
  L3GInitStructure.xEndianness = L3G_LITTLE_ENDIAN;

  /* Configure the gyro main parameters */
  L3gd20Config(&L3GInitStructure);

  /* Configure the MCU exti */
  ExtiConfiguration();

  /* Configure the sensor data ready */
  L3gd20Irq2Config(L3G_I2_DRDY,L3G_ENABLE);

  while(1)
  {
    /* Wait for data ready (set by the proper ISR) */
    while(!xDataReady);
    xDataReady=RESET;

    /* Read data */
    L3gd20ReadAngRate(fGyroXYZ);

#ifdef USE_VCOM
    /* Print data via Virtual Com */
    Stm32f1VCPrintf("X: %f , Y:%f , Z:%f\n\r", fGyroXYZ[0],fGyroXYZ[1],fGyroXYZ[2]);
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
