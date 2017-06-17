/**
 * @file    UsbGatekeeper.c
 * @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
 * @version V1.0.1
 * @date    01 February 2013
 * @brief   USB management module for iNemo. This file contains the implementation of the
 *          function prototypes of <i>iNemoGatekeeper.h</i>.
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


/* Includes ------------------------------------------------------------------*/
#include "iNemoGatekeeper.h"
#include "STM32F1_VC_General.h"


/**
 * @addtogroup iNEMO_Discovery_System
 * @{
 */


/**
 * @defgroup iNemo_Gatekeeper         iNemo Gatekeeper
 * @{
 */


/**
 * @brief USB frame buffer (imported from the Virtual Com Port Driver).
 */
extern uint8_t g_vectcVCRxBuffer[VIRTUAL_COM_PORT_DATA_SIZE];


/**
 * @brief USB frame size (imported from the Virtual Com Port Driver).
 */
extern uint32_t g_lVCNbBytesReceived;


/**
 * @brief This resource is used to synchronize the USB ISR and the UsbSendData function to the UsbGatekeeper task.
 */
static xQueueHandle s_xUsbISR2GKQueue;


/**
 * @brief This resource is used to forward the frames to the FrameParser task.
 */
xQueueHandle xiNemoGK2ParserQueue;


/**
 * @brief The task to manage the coming and going of the USB frames.
 */
static void iNemoGateKeeperTask(void *pvParameters);


/**
 * @defgroup iNEMO_Gatekeeper_Functions                     iNemo USB Gatekeeper Functions
 * @{
 */


/**
 * @brief Initialization of the hardware used for the USB Gatekeeper task.
 * @retval None.
 */
void iNemoGateKeeperTaskHwSetup(void)
{
  /* Virtual Com initialization */
  Stm32f1VCInit();

}


/**
 * @brief The function that create the task and the resources used.
 * @param pvParameters: The parameter passed into the task.
 * @retval None.
 */
void iNemoGateKeeperTaskCreate(void *pvParameters)
{
  /* Create the queue used to synchronize the USB ISR and the UsbSendData function to the UsbGatekeeper task */
  s_xUsbISR2GKQueue = xQueueCreate( 15, sizeof(iNemoFrame) );
  if (!s_xUsbISR2GKQueue)
    /* Error in resource creation. */
    for(;;);

  /* Create the queue used to send the frames to the FrameParserTaskCreate */
  xiNemoGK2ParserQueue = xQueueCreate( 10, sizeof(iNemoFrame) );
  if (!xiNemoGK2ParserQueue)
    /* Error in resource creation. */
    for(;;);

  /* Create the UsbGatekeeperTask task */
  if ( xTaskCreate( iNemoGateKeeperTask, "USB GATEKEEPER", configMINIMAL_STACK_SIZE, NULL, 4, NULL ) != pdPASS )  //( signed portCHAR * )
    /* Error in task creation */
    for(;;);

}


/**
 * @brief The task to manage the frames received or to send by USB port.
 * @param pvParameters: The parameter passed into the task.
 * @retval None.
 */
static void iNemoGateKeeperTask(void *pvParameters)
{
  /* The structure that contains the frame, its length and the direction */
  static iNemoFrame xUsbFrameParse;

  /* The task is implemented in an infinite loop */
  for(;;)
    /*
    * Use the queue to wait the events "frame received from usb" or "frame to send by usb port".
    * The second parameter is the structure that contains the frame and its information.
    * This task will remain in the blocked state to wait for data to be available.
    */
    if(xQueueReceive(s_xUsbISR2GKQueue, &xUsbFrameParse, portMAX_DELAY) == pdTRUE)
    {
      /* Checks if the frame in the queue is an input frame, then sends the frame to the FrameParser task */
      if(xUsbFrameParse.uciNemoIOFrame)
      {
        xQueueSendToBack(xiNemoGK2ParserQueue, &xUsbFrameParse, 0);
      }
      else /* Else, the frame in the queue is an output frame */
      {
        /* Send the data by the virtual com */
        Stm32f1VCWriteTxBuffer(xUsbFrameParse.uciNemoFrame, xUsbFrameParse.uciNemoFrameLength);
        Stm32f1VCSendData();
      }
    }

}


/**
 * @brief  Send the output frame to the USB Gatekeeper task.
 * @param  ucFrame the buffer with the frame to send.
 * @param  ucFrameLength the length of the frame.
 * @retval None.
 */
void iNemoGKSendData(uint8_t* ucFrame, uint8_t ucFrameLength)
{
  static iNemoFrame xUsbFrameParseOutput;

  /* Fill the structure with all the needed information */
  xUsbFrameParseOutput.uciNemoFrameLength = ucFrameLength;
  for(uint16_t i=0;i<ucFrameLength;i++)
    xUsbFrameParseOutput.uciNemoFrame[i] = ucFrame[i];

  /* Set as output frame */
  xUsbFrameParseOutput.uciNemoIOFrame = OUTPUT_FRAME;

  /* Forward the frame to the UsbGatekeeper task */
  xQueueSendToBack(s_xUsbISR2GKQueue, &xUsbFrameParseOutput, portMAX_DELAY);

}


/**
 * @brief The USB ISR.
 * @retval None.
 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* The STR events interrupt service routine. */
  Stm32f1VCIntServRoutine();

  /* Checks if at least three bytes have been received. */
  if(g_lVCNbBytesReceived>=3)
  {
    /* Fill the structure with all the needed information */
    iNemoFrame xUsbFrameParse;
    xUsbFrameParse.uciNemoFrameLength = g_lVCNbBytesReceived;
    for(int i=0;i<g_lVCNbBytesReceived;i++)
      xUsbFrameParse.uciNemoFrame[i] = g_vectcVCRxBuffer[i];

    /* Declares the received frame as input frame */
    xUsbFrameParse.uciNemoIOFrame = INPUT_FRAME;

    /* The higher priority task woken variable used in the xQueueSendFromISR function */
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    /* Sends an item into the queue */
    xQueueSendFromISR(s_xUsbISR2GKQueue, &xUsbFrameParse, &xHigherPriorityTaskWoken);

    /* Resets the variable g_lVCNbBytesReceived, so it is ready to a next received frame */
    g_lVCNbBytesReceived = 0;

    /* If the xHigherPriorityTaskWoken is set to pdTRUE then a context switch shoud be performed */
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
  }

}


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
