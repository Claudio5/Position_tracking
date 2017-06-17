/**
  * @file    iNemo.c
  * @author  ART Team IMS-Systems Lab
  * @version V2.3.0
  * @date    01 February 2013
  * @brief   This file defines the iNEMO Tasks.
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


#include "iNemo.h"
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "iNemoHandlers.h"
#include "utils.h"
#include "iNemoLib.h"
#include "iNemoGatekeeper.h"

/**
 * @addtogroup iNEMO_Discovery_System             iNemo Discovery System
 * @brief	Discovery System Core.
 * 			The whole system is divided into some modules managing different parts.
 * @{
 */

/**
 * @addtogroup iNEMO_Core                         iNemo Core
 * @brief This is the main iNemo discovery system module. It contains the Command and Data task implementations.
 * @{
 */

/**
 * @brief This resource is used to synchronize the USB IRQ and the iNemo Command Manager task.
 */
extern xQueueHandle xiNemoGK2ParserQueue;

/**
 * @brief This resource is used to synchronize the TIM2 IRQ and the iNemo data acquisition task.
 */
static xSemaphoreHandle s_timSemaphore;

/**
 * @brief This resource is used to protect the data buffer when an asyncronous DATA_REQUEST_FRAME is received.
 */
static xSemaphoreHandle s_xDataMutex;

/**
 * @addtogroup iNEMO_Core_TypesDefinitions                      iNemo Core Types Definitions
 * @{
 */

/**
 * @brief This define maps the length of frame used in the iNEMO USB protocol.
 */
#define FRAME_MAP_LENGTH  25

/**
 * @brief This array is used to access the services requested by a frame.
 *        Every time a new frame is received its ID is extracted and searched among the ones in
 *        this array. The associated handler function is called as soon as the match happens.
 */
static const FrameMap s_xiNemoFrameActionMap[FRAME_MAP_LENGTH]={
  {iNEMO_Connect, iNEMO_Connect_Length, iNemoConnectHandler},
  {iNEMO_Disconnect, iNEMO_Disconnect_Length, iNemoDisconnectHandler},
  {iNEMO_Reset_Board, iNEMO_Reset_Board_Length, iNemoResetBoardHandler},
  {iNEMO_Enter_DFU_Mode, iNEMO_Enter_DFU_Mode_Length, iNemoEnterDfuModeHandler},
  {iNEMO_Led, iNEMO_Led_Length, iNemoLedHandler},
  {iNEMO_Get_Device_Mode, iNEMO_Get_Device_Mode_Length, iNemoGetDeviceModeHandler},
  {iNEMO_Get_MCU_ID, iNEMO_Get_MCU_ID_Length, iNemoGetMcuIdHandler},
  {iNEMO_Get_FW_Version, iNEMO_Get_FW_Version_Length, iNemoGetFwVersionHandler},
  {iNEMO_Get_HW_Version, iNEMO_Get_HW_Version_Length, iNemoGetHwVersionHandler},
  {iNEMO_Identify, iNEMO_Identify_Length, iNemoIdentifyHandler},
  {iNEMO_Get_AHRS_Library, iNEMO_Get_AHRS_Library_Length, iNemoGetAhrsLibraryHandler},
  {iNEMO_Get_Libraries, iNEMO_Get_Libraries_Length ,iNemoGetLibrariesHandler},
  {iNEMO_Get_Available_Sensors, iNEMO_Get_Available_Sensors_Length ,iNemoGetAvailSensorsHandler},
  {iNEMO_SetOutMode, iNEMO_SetOutMode_Length ,iNemoSetOutModeHandler},
  {iNEMO_GetOutMode,iNEMO_GetOutMode_Length, iNemoGetOutModeHandler},
  {iNEMO_Set_Sensor_Parameter, iNEMO_Set_Sensor_Parameter_Length, iNemoSetSensorParameterHandler},
  {iNEMO_Get_Sensor_Parameter, iNEMO_Get_Sensor_Parameter_Length, iNemoGetSensorParameterHandler},
  {iNEMO_Restore_Default_Parameter, iNEMO_Restore_Default_Parameter_Length, iNemoRestoreDefaultParameterHandler},
  {iNEMO_Start_Acquisition, iNEMO_Start_Acquisition_Length, iNemoStartAcquisitionHandler},
  {iNEMO_Stop_Acquisition, iNEMO_Stop_Acquisition_Length, iNemoStopAcquisitionHandler},
  {iNEMO_Get_Acq_Data, iNEMO_Get_Acq_Data_Length, iNemoGetAcqDataHandler},
  {iNEMO_Start_HIC_Calibration, iNEMO_Start_HIC_Calibration_Length, iNemoStartHICCalibrationHandler},
  {iNEMO_Stop_HIC_Calibration, iNEMO_Stop_HIC_Calibration_Length, iNemoStopHICCalibrationHandler},
  {iNEMO_Save_to_Flash, iNEMO_Save_to_Flash_Length, iNemoSaveToFlashHandler},
  {iNEMO_Load_from_Flash, iNEMO_Load_from_Flash_Length, iNemoLoadFromFlashHandler},

};

/**
 *@}
 */


/**
 * @addtogroup iNEMO_Core_function                      iNemo Core Functions
 * @{
 */



/**
 * @brief Configures the timer 2 to raise the update IRQ every 1/nFrequency sec.
 * @param  fFrequency: Desired frequency in Hz.
 *         This parameter is a float.
 */
void iNemoTimerConfig(float fFrequency)
{
  static uint16_t nPrescaler,nPeriod;
  uint32_t lNClkCycles;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

   //Enable timer clocks
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );

  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

  TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );

  // Time base configuration for timer 2 - which generates the interrupts.
  lNClkCycles = (uint32_t)(configCPU_CLOCK_HZ/fFrequency);

  // compute the counter and prescaler
  TimerFindFactors(lNClkCycles, &nPrescaler, &nPeriod);
  TIM_TimeBaseStructure.TIM_Period = nPeriod - 1;
  TIM_TimeBaseStructure.TIM_Prescaler = nPrescaler - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure );
  TIM_ARRPreloadConfig( TIM2, ENABLE );

  // configure the NVIC for the TIM2 IRQ
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( &NVIC_InitStructure );

}

/**
 * @brief Implement a generic interface for an user defined task.
 * @details It is called by the prvSetupHardware function to setup all hardware resource used by the task.
 * @param pvParameters :  not used
 */
void iNemoInitTask(void *pvParameters)
{
  /* Configure the gatekeeper task */
  iNemoGateKeeperTaskHwSetup();

  /* Configure the iNemo hardware */
  iNemoHwConfig();

  /* Configure sensors */
  iNemoSensorsConfig();

   /* Detect the M1 board version */
  iNemoBoardRecognition();
  
  /* Configure the timer for sampling (50Hz default) */
  iNemoTimerConfig(50.0);

}


/**
 * @brief iNemo Command Manager task control function.
 * @details This task waits for a frame coming from the USB peripheral.
 * @param pvParameters not used.
 */
void iNemoCommandTaskFunction(void *pvParameters) {

  /* The structure that contains the frame and its length to parse */
  iNemoFrame xiNemoFrameParse;

  /* If this flag is 1, the task will expect only other frame with the same ID */
  uint8_t ucMoreFragmentFlag = 0;

  /* The buffer to build the main frame starting from the MF frames. */
  uint8_t ucMoreFragmentFrame[260];
  uint8_t ucMoreFragmentIndex = 3;

  for (;;)
    /*
    * Use the queue to wait for the event "frame received from usb".
    * The second parameter is the structure that contains the frame and its information.
    * This task will remain in the blocked state to wait for data to be available.
    */
    if (xQueueReceive(xiNemoGK2ParserQueue, &xiNemoFrameParse, portMAX_DELAY) == pdTRUE)
    {
      /* Check if the length of the frame matches with the length of the data received from usb. */
      if((xiNemoFrameParse.uciNemoFrame[1] + 2) == xiNemoFrameParse.uciNemoFrameLength)
      {
        /* Check if the device is not connected and the frame is not iNEMO_Connect. */
        if(!iNemoGetConnectionState() && xiNemoFrameParse.uciNemoFrame[2] != iNEMO_Connect && (xiNemoFrameParse.uciNemoFrame[0] & 0x20))
        {
          /* In this case a nack frame WrongSyntax is sent. */
          iNemoSendNack(iNEMO_Connect,WrongSyntax);
        }
        /* Else, the frame is a iNEMO_Connect or the device is connected. */
        else
        {
          uint8_t cCmdNotSupported = 1;

          /* Check if the frame is a more fragment */
          if(xiNemoFrameParse.uciNemoFrame[0] & 0x10)
          {
            /* Copy the frame into the buffer */
            for(uint8_t ucI = 0; ucI < (xiNemoFrameParse.uciNemoFrameLength - 3); ucI++)
              ucMoreFragmentFrame[ucMoreFragmentIndex + ucI] = xiNemoFrameParse.uciNemoFrame[3 + ucI];
            ucMoreFragmentIndex += (xiNemoFrameParse.uciNemoFrameLength - 3);

            /* Resets the flag because a frame handler function for the received frame has been found. */
            cCmdNotSupported = 0;

            ucMoreFragmentFlag = 1;
          }
          else if(ucMoreFragmentFlag) /* If the flag is 1, this frame is the LF frame that concludes the more fragment frames. */
            {
              /* Build the main frame */
              ucMoreFragmentFrame[0] = xiNemoFrameParse.uciNemoFrame[0];
              ucMoreFragmentFrame[2] = xiNemoFrameParse.uciNemoFrame[2];

              for (uint8_t ucI = 0; ucI < (xiNemoFrameParse.uciNemoFrameLength - 3); ucI++)
                ucMoreFragmentFrame[ucMoreFragmentIndex + ucI] = xiNemoFrameParse.uciNemoFrame[3 + ucI];

              ucMoreFragmentIndex += (xiNemoFrameParse.uciNemoFrameLength - 3);
              ucMoreFragmentFrame[1] = ucMoreFragmentIndex - 3 + 1;

              ucMoreFragmentIndex = 3;
              ucMoreFragmentFlag = 0;

              /* Searches if the frame ID (of the received frame) matches with a known one into the map with the iNemo frame. */
              for(uint32_t i = 0; i < FRAME_MAP_LENGTH; i++)
                if (s_xiNemoFrameActionMap[i].ucFrameId == xiNemoFrameParse.uciNemoFrame[2])
                {
                  /* Resets the flag because a frame handler function for the received frame has been found. */
                  cCmdNotSupported = 0;

                  /* Call the handler matched by the frame ID */
                  s_xiNemoFrameActionMap[i].FrameHandler(ucMoreFragmentFrame);

                  break;
                }
            }
          /* If the frame is just a LF frame, this task have only the job to call the function that handles the frame. */
          else
          {
            /* Searches if the frame ID (of the received frame) matches with a known one into the map with the iNemo frame. */
            for(uint32_t i = 0; i < FRAME_MAP_LENGTH; i++)
              if (s_xiNemoFrameActionMap[i].ucFrameId == xiNemoFrameParse.uciNemoFrame[2] && (s_xiNemoFrameActionMap[i].ucFrameLength==0 || s_xiNemoFrameActionMap[i].ucFrameLength == xiNemoFrameParse.uciNemoFrame[1]))
              {
                /* Resets the flag because a frame handler function for the received frame has been found. */
                cCmdNotSupported = 0;

                if(s_xiNemoFrameActionMap[i].ucFrameId==0x54)
                  xSemaphoreTake(s_xDataMutex,portMAX_DELAY);

                /* Call the handler matched by the frame ID */
                s_xiNemoFrameActionMap[i].FrameHandler(xiNemoFrameParse.uciNemoFrame);

                if(s_xiNemoFrameActionMap[i].ucFrameId==0x54)
                  xSemaphoreGive(s_xDataMutex);

                break;
              }
          }

          /* If the flag is still set, this means that no frame handler function has been found and so that frame ID is not supported. */
          if (cCmdNotSupported && (xiNemoFrameParse.uciNemoFrame[0] & 0x20))
            /* Calls the function that send the CommandNotSupported nack frame. */
            iNemoSendNack(xiNemoFrameParse.uciNemoFrame[2], CmdUnsupported);
        }

      }
      else if(xiNemoFrameParse.uciNemoFrame[0] & 0x20)
        /* In this case a nack frame WrongSyntax is sent. */
        iNemoSendNack(xiNemoFrameParse.uciNemoFrame[2],WrongSyntax);

    }

}

/**
 * @brief iNemo data acquisition/send task control function.
 * @details This task is synchronized with the  Timer2 interrupt. It reads all sensor data, packages the data according to the frame
 * format and sends the data to the PC over the USB connection.
 * @param pvParameters not used.
 */
void iNemoDataTaskFunction(void *pvParameters)
{

  /* Init the data struct */
  iNemoInitData();

  for (;;) {
    /* wait for the timer sampling event */
    if ( xSemaphoreTake(s_timSemaphore, portMAX_DELAY) == pdTRUE ) {

      xSemaphoreTake(s_xDataMutex,portMAX_DELAY);

      /* Sample data according to the user request and send them */
      iNemoDataProcess();

      xSemaphoreGive(s_xDataMutex);
    }
  }

}


/**
 * @brief Implement a generic interface for an user defined task.
 * @details It is called by the main to allocate the task's software resources.
 * @param pvParameters not used.
 */
void iNemoStartTask(void *pvParameters)
{
  /* Create the gatekeeper. This task will be responsible to deal with the communication peripherial. */
  iNemoGateKeeperTaskCreate(NULL);

  /* DAta mutex */
  s_xDataMutex=xSemaphoreCreateMutex();
  xSemaphoreGive(s_xDataMutex);

  /* Create the semaphore used to synchronize  the iNemo data acquisition task and the TIM2 interrupt service routine. */
  vSemaphoreCreateBinary(s_timSemaphore);
  if (!s_timSemaphore) {
    /* Error in resource creation. */
    while (1);
  }
  xSemaphoreTake(s_timSemaphore, 0);

  /* Create the iNemo Command task */
  if ( xTaskCreate(iNemoCommandTaskFunction, "iNemoCmd", IN_TASK_STACK_SIZE, NULL, IN_TASK_PRIORITY+1, NULL) != pdPASS ) {
    /* Error in task creation */
    while (1);
  }

  /* Create the iNemo Data stream task */
  if ( xTaskCreate(iNemoDataTaskFunction, "iNemoData", IN_TASK_STACK_SIZE, NULL, IN_TASK_PRIORITY, NULL) != pdPASS ) {
    /* Error in task creation */
    while (1);
  }

}



/**
 * @brief This function handles TIM2 global interrupt request by resuming the
 * iNemoData task.
 */
/*void TIM2_IRQHandler(void)
{


  if(TIM_GetITStatus(TIM2, TIM_IT_Update))
  {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

   //  Communicate with the Data process task
    xSemaphoreGiveFromISR(s_timSemaphore, &xHigherPriorityTaskWoken);

     //Clear the IRQ bit
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

     //Force a context switch if the task on the queue has an higher priority than the current running one
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

  }


}*/

#if (defined(_GYRO) || defined(_6X))
/**
 * @brief  This function handles External interrupt request (associated with accelerometer and gyroscope data ready line).
 */
void EXTI9_5_IRQHandler(void)
{
  /* accelerometer IRQ line */
  if(EXTI_GetITStatus(EXTI_Line5) != RESET || EXTI_GetITStatus(EXTI_Line6) != RESET || EXTI_GetITStatus(EXTI_Line7) != RESET || EXTI_GetITStatus(EXTI_Line8) != RESET)
  {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    /* Communicate with the Data process task */
    xSemaphoreGiveFromISR(s_timSemaphore, &xHigherPriorityTaskWoken);

    /* accelerometer (in old ver) IRQ line */
    if(EXTI_GetITStatus(EXTI_Line5) != RESET)
      /* Clear the pending bit */
      EXTI_ClearITPendingBit(EXTI_Line5);

    /* gyro IRQ line */
    if(EXTI_GetITStatus(EXTI_Line6) != RESET)
      /* Clear the pending bit */
      EXTI_ClearITPendingBit(EXTI_Line6);

    /* magnetometer (in new ver) IRQ line */
    if(EXTI_GetITStatus(EXTI_Line7) != RESET)
      /* Clear the pending bit */
      EXTI_ClearITPendingBit(EXTI_Line7);
    
    /* accelerometer (in new ver) IRQ line */
    if(EXTI_GetITStatus(EXTI_Line8) != RESET)
      /* Clear the pending bit */
      EXTI_ClearITPendingBit(EXTI_Line8);
    
    /* Force a context switch if the task on the queue has an higher priority than the current running one */
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);


  }

}
#endif


#ifdef _6X
/**
 * @brief  This function handles External interrupt request (associated with magnetometer (HW old ver) data ready line).
 */
void EXTI4_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line4) != RESET)
  {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    /* Communicate with the Data process task */
    xSemaphoreGiveFromISR(s_timSemaphore, &xHigherPriorityTaskWoken);

    /* Clear the pending bit */
    EXTI_ClearITPendingBit(EXTI_Line4);

    /* Force a context switch if the task on the queue has an higher priority than the current running one */
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

  }

}
#endif


#ifdef _PRESS
/**
 * @brief  This function handles External interrupt request (associated with pressure sensor data ready line).
 */
void EXTI0_IRQHandler(void)
{
  /* pressure IRQ line */
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    /* Communicate with the Data process task */
    xSemaphoreGiveFromISR(s_timSemaphore, &xHigherPriorityTaskWoken);

    /* Clear the pending bit */
    EXTI_ClearITPendingBit(EXTI_Line0);

    /* Force a context switch if the task on the queue has an higher priority than the current running one */
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
  }

}
#endif

/*
 * @}
 */

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
