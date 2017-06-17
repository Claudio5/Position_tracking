/**
  * @file    main.c
  * @author  ART Team IMS-Systems Lab
  * @version V2.0.0
  * @date    09/20/2010
  * @brief   iNEMO Device Firmware Upgrade (DFU) main program body.
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
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "iNEMO_DFU_General.h"
#include "iNEMO_DFU_Prop.h"
#include "iNEMO_Led.h"
#include "iNEMO_Button.h"

/** @defgroup iNEMO_DFU_Program
  * @{
  */

/** @defgroup iNEMO_DFU_Application
  * @{
  */


/* Private typedef -----------------------------------------------------------*/
typedef  void (*pFunction)(void);

/* Private define ------------------------------------------------------------*/


/* Option Byte address from where read and write value to enter or leave DFU mode */
#define Option_address ((uint32_t)0x1FFFF804)


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
uint8_t DeviceState;
uint8_t DeviceStatus[6];
pFunction Jump_To_Application;
uint32_t JumpAddress;


/* Private function prototypes -----------------------------------------------*/



/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Reads the Option Byte to enter DFU in Software Mode
  * @param  None
  * @retval Option Byte Value
  */
uint8_t Option_Byte_Read (void)
{
  return (uint8_t) (*(__IO uint32_t*) Option_address);
}


/**
  * @brief  Program the Option Byte to leave DFU in Software Mode
  * @param  none.
  * @retval None
  */
void Leave_DFU_SW_Mode(void)
{
  /* Unlock the Flash Program Erase controller */
  FLASH_Unlock();
  FLASH_EraseOptionBytes();  
  FLASH_ProgramOptionByteData(Option_address, 0x00);
  FLASH_Lock();
  
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
    /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */
  
  iNEMO_Button_Init(BUTTON_KEY, BUTTON_MODE_GPIO);

  /* Check if the Key push-button on iNEMO Board is pressed or the Option Byte is 0*/
   if(!(!iNEMO_Button_Get_State(BUTTON_KEY) || Option_Byte_Read() == 0x01))
  { /* Test if user code is programmed starting from address 0x8003000 */
    if (((*(__IO uint32_t*)ApplicationAddress) & 0x2FFE0000 ) == 0x20000000)
    { /* Jump to user application */

      JumpAddress = *(__IO uint32_t*) (ApplicationAddress + 4);
      Jump_To_Application = (pFunction) JumpAddress;
      /* Initialize user application's Stack Pointer */
      __set_MSP(*(__IO uint32_t*) ApplicationAddress);
      Jump_To_Application();
    }
  } /*!< Otherwise enters DFU mode to allow user to program his application */


  /*Delete the Option Byte*/
  if(Option_Byte_Read()==0x01)
  {
    Leave_DFU_SW_Mode();
  }

  /* Enter DFU mode */
  DeviceState = STATE_dfuERROR;
  DeviceStatus[0] = STATUS_ERRFIRMWARE;
  DeviceStatus[4] = DeviceState;

  /* iNemo LED initialization */
  iNEMO_Led_Init(LED1);
  
  /* DFU initialization */
  iNemoDFUInit();

  /* Main loop */
  while (1)
  {
    for(volatile uint32_t i=0;i<0x2FFFFF;i++);
    
    iNEMO_Led_Toggle(LED1);
  }
}






#ifdef USE_FULL_ASSERT
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
  */ /* end of group iNEMO_DFU_Application */

/**
  * @}
  */ /* end of group iNEMO_DFU_Program */


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
