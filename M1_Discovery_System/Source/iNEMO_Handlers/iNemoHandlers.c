/**
 *
 * @file     iNemoHandlers.c
 * @author   IMS Systems Lab - ART Team
 * @version  V2.3.0
 * @date    01 February 2013
 * @brief    This file provides all the API between the Application and PC serial communication.
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

#include "iNemoHandlers.h"
#include "string.h"
#include "utils.h"
#include "math.h"
#include "iNemo.h"
#include <stdio.h>

/**
 * @addtogroup iNEMO_Discovery_System
 * @{
 */

/**
 * @addtogroup iNEMO_Handlers                          iNemo Handlers
 * @brief This module includes all the features to manage the protocol frames. The frames are
 *        exchanged with the host through the gatekeeper wich accesses the communication peripherial.
 * @{
 */

/**
 * @addtogroup iNEMO_Handlers_Constants                iNemo Handlers Constants
 * @{
 */


/**
 * @brief Max frame dimension 
 */
#define MAX_FRAME_DIM 96


/**
 * @}
 */


/**
 * @addtogroup iNEMO_Handlers_Private_Types                iNemo Handlers Private Types
 * @brief Useful types definitions for the GUI layer.
 * @{
 */

/**
 * @brief This data structure contains the pointers to the functions to be called in the DataProcess handler
 * in the order in which they have to be called.
 */
typedef struct{
  uint8_t cNDataProcActions;                                    /*!< length of the action pointer array */
  void (*xDataProcessActions[10])(uint8_t *pcDataBuffer, uint8_t *pcNByteToSend);   /*!< actions pointer array */
  uint8_t cResponseFrameId;                                     /*!< ID to be used for the response frame */
} iNemoDataProcessActions;


/**
 * @}
 */

/**
 * @addtogroup iNEMO_Handlers_Private_Variables                iNemo Handlers Private Variables
 * @brief GUI layer private variables to store data and some kind of states.
 * @{
 */

/**
 * @brief Specifies the incremental counter. Defines the base time.
 */
static uint16_t s_iC = 0;


/**
 * @brief Number of sample to be acquired
 */
static uint16_t s_nLengthSample = 0;


/**
 *  @brief specifies which output data are enabled
 *  @details: 0 Disable/1 Enable order bit :AHRS | RFU 0 | Cal/Raw | Accelerometer | Gyro | Magnetometer | Pressure | Temperature
 *   Cal/Raw   0-> Calibrated data
 *                 1-> Raw data [lsb]
 */
static uint8_t s_uOutSelect = 0x1F;


/**
 *  @brief specifies comunicatin frequency and output channel
 *  @details :RFU 0 | RFU 0 | FQ2 | FQ1 | FQ0 | OT2 | OT1 | OT0
 *      FQ : 000 (1Hz); 001 (10Hz); 010 (25Hz); 011 (50Hz)
 *      OT : 000 (USB output)
 */
static uint8_t s_uOutMode=0;

/**
 *  @brief specifies if the iNEMO is connected
 */
static bool s_bConnectState=FALSE;


/**
 *  @brief specifies the iNEMO is working in Sensor Mode only
 */
static uint8_t s_uDeviceMode=0x00;


/**
 *  @brief specifies if trace is enabled
 */
static bool s_bTraceEnabled=FALSE;


/**
 *  @brief sensor sampling frequency
 */
static uint8_t s_uTimerFrequence=0x03;


/** 
 * @brief Specifies if the raw sensor data sending is enabled.
 */
static bool s_bRawDataEnabled = FALSE;


/** 
 * @brief Specifies if the ASK_DATA_MODE is enabled.
 * @details In this mode (flag set to TRUE) the board will only send the acquired data only
 *          when the iNEMO_Get_Acq_Data cmd arises.
 */
static bool s_bAskDataMode = FALSE;


/**
 * @brief iNemoData struct shared beetwen data and command task.
 *        This resouce is not protected because all tasks have to modify it in a
 *        mutual exclusive way.
 */
static iNemoData s_xDataStruct;

/**
 * @brief Number of data to be sent
 */
static uint8_t cNByteToSend;

/**
 * @brief iNemoData formatted for transmission 
 */
static uint8_t pDataBuffer[100]= {0};

/**
 * @brief Main data process actions variable.  
 */
static iNemoDataProcessActions s_xDataProcActStruct;

/**
 * @brief Second data process actions variable in wich save the main data process
 * when other actions have to be performed for short time periods.
 */
static iNemoDataProcessActions s_xDataProcActStructSave;

#if defined(AHRS_MOD) && defined(AHRS_JUMP)
/**
 * @brief Dummy array used to protect the AHRS_RAM_SIZE bytes in RAM starting from the 0x20000000 address.
 *        This instance ensures that this application won't allocate any variable in the memory segment used 
 *        by the AHRS_Lib.
 * @note  The user should not write this array in any way from this application.
 */
volatile uint8_t dummyFill4Ahrs[AHRS_RESERVED_RAM_SIZE] @ AHRS_RESERVED_RAM_START_ADDR;

/**
 * @brief AHRS function pointers (to be initialized using the @ref AHRS_POINTERS_INIT macro.
 */
pAHRSFunction iNEMO_AHRS_Init,iNEMO_AHRS_DeInit,iNEMO_AHRS_Update;
#endif

/**
 * @}
 */


/**
 * @addtogroup iNEMO_Handlers_Functions                         iNemo Handlers Functions
 * @{
 */

static void iNemoEnableTimer(FunctionalState command);


/**
 * @defgroup iNEMO_Handlers_Data_Process_Mng                         Data Process Management
 * @brief These functions must be used to manage the actions list structure.
 * @note It is strongly recommended not to use other ways to modify @ref s_xDataProcActStruct and @ref s_xDataProcActStructSave .
 * @{
 */

/**
 * @brief  Appends a new function to the actions array.
 * @param  xProcActToAdd: pointer to the function to append.
 *         This function must be a prototype like void function(uint8_t*,uint8_t*)
 * @retval None.
 */
void iNemoDataProcessAppend(void (*xProcActToAdd)(uint8_t *pcDataBuffer, uint8_t *pcNByteToSend))
{
  for(uint8_t i=0 ; i<s_xDataProcActStruct.cNDataProcActions ; i++)
    if(s_xDataProcActStruct.xDataProcessActions[i] == xProcActToAdd)
      return;
  
  s_xDataProcActStruct.xDataProcessActions[s_xDataProcActStruct.cNDataProcActions] = xProcActToAdd;
  
  s_xDataProcActStruct.cNDataProcActions++;
  
}


/**
 * @brief  Clears the action functions array making a save copy before.
 * @retval None.
 */
void iNemoDataProcessClear(void)
{
  /* Store the old struct before deleting */
  s_xDataProcActStructSave = s_xDataProcActStruct;
   
  /* Delete the old buffer */
  s_xDataProcActStruct.cNDataProcActions=0;
  
}


/**
 * @brief  Restores the action functions array saved before the clear.
 * @retval None.
 */
void iNemoDataProcessRestore(void)
{
  s_xDataProcActStruct = s_xDataProcActStructSave;
  
}


/**
 * @brief  Allows to set the frame id to user for the transmitted frame.
 * @param  cFrameId: frame ID.
 * @retval None.
 */
void iNemoDataProcessSetResponseFrameId(uint8_t cFrameId)
{
  s_xDataProcActStruct.cResponseFrameId=cFrameId;
  
}


/**
 * @}
 */


/**
 * @defgroup iNEMO_Handlers_Handlers                Frame Command Handlers
 * @{
 */

/**
 * @brief  Connect command handler.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoConnectHandler(uint8_t* pUsbFrame)
{
  if(pUsbFrame[0] & 0x20)
    iNemoSendAck(iNEMO_Connect,0,0x00);
  
  s_bConnectState = TRUE;
  iNEMO_Led_On(LED1); 
}


/**
 * @brief  Disconnect command handler.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoDisconnectHandler(uint8_t* pUsbFrame)
{
  if(pUsbFrame[0] & 0x20)
    iNemoSendAck(iNEMO_Disconnect,0,0x00);
  
  s_bConnectState=FALSE;
  iNEMO_Led_Off(LED1);

}


/**
 * @brief  Reset board command handler.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoResetBoardHandler(uint8_t* pUsbFrame)
{
  if(pUsbFrame[0] & 0x20)
    iNemoSendAck(iNEMO_Reset_Board,0,0x00);
  
  vTaskDelay(5000 / portTICK_RATE_MS); /*delay to allow the GUI receive ACK*/
  s_bConnectState=FALSE;

  NVIC_SystemReset();
}


/**
 * @brief  Enter DFU command handler.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoEnterDfuModeHandler(uint8_t* pUsbFrame)
{
  if(pUsbFrame[0] & 0x20)
    iNemoSendAck(iNEMO_Enter_DFU_Mode,0,0x00);

  /* Unlock the Flash Program Erase controller */
  FLASH_Unlock();
  FLASH_EraseOptionBytes();
  FLASH_ProgramOptionByteData(OPTION_ADDRESS, 0x01);
  FLASH_Lock();

  /* Reset device */
  NVIC_SystemReset();

}


/**
 * @brief  Led command handler.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoLedHandler(uint8_t* pUsbFrame)
{
  /* check the LED ON/OFF frame field */
  if(pUsbFrame[3])
    iNEMO_Led_On(LED1); 
  else
    iNEMO_Led_Off(LED1);

  if(pUsbFrame[0] & 0x20)
    iNemoSendAck(iNEMO_Led,0,0x00);

}


/**
 * @brief  Get device command handler.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoGetDeviceModeHandler(uint8_t* pUsbFrame)
{
  if(pUsbFrame[0] & 0x20)
    iNemoSendAck(iNEMO_Get_Device_Mode,1,&s_uDeviceMode);
}


/**
 * @brief  Get MCU ID command handler.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoGetMcuIdHandler(uint8_t* pUsbFrame)
{
  uint8_t mcu_id[12];

  for (uint8_t i=0; i<12; ++i)
    mcu_id[i] = MCU_ID[12-(i+1)];
  
  if(pUsbFrame[0] & 0x20)
    iNemoSendAck(iNEMO_Get_MCU_ID, 12, mcu_id);
}


/**
 * @brief  Get Firmware version command handler.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoGetFwVersionHandler(uint8_t* pUsbFrame)
{
  if(pUsbFrame[0] & 0x20)
    iNemoSendAck(iNEMO_Get_FW_Version, (SIZE_FWversion), iNEMO_FIRMWARE_VERSION);

}


/**
 * @brief  Get Hardware version command handler.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoGetHwVersionHandler(uint8_t* pUsbFrame)
{
  if(pUsbFrame[0] & 0x20)
    iNemoSendAck(iNEMO_Get_HW_Version, (SIZE_HWversion), iNEMO_HARDWARE_VERSION);
}


/**
 * @brief  Identify command handler.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoIdentifyHandler(uint8_t* pUsbFrame)
{
  uint8_t mcu_id[12];

  for (int i=0; i<12; i++)
    mcu_id[i] = MCU_ID[12-(i+1)];
  
  if(pUsbFrame[0] & 0x20)
    iNemoSendAck(iNEMO_Identify, 12, mcu_id);
  
  for (uint8_t i=0; i<8; i++)
  {
    iNEMO_Led_Toggle(LED1);
    vTaskDelay(500 / portTICK_RATE_MS);
  }
}


/**
 * @brief  Get AHRS library version command handler.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoGetAhrsLibraryHandler(uint8_t* pUsbFrame)
{
  if(pUsbFrame[0] & 0x20)
  {
    char striNEMO_AHRS_VERSION[30];
    
    if(iNEMO_AHRS_LIBRARY)
    {
      sprintf(striNEMO_AHRS_VERSION,"iNEMO AHRS V%d.%d.%d",iNEMO_AHRS_VERSION_N,iNEMO_AHRS_VERSION_SUBN,iNEMO_AHRS_VERSION_SUBMIN);
    }
    else
    {
      sprintf(striNEMO_AHRS_VERSION,"iNEMO AHRS NOT AVAILABLE");
    }
    iNemoSendAck(iNEMO_Get_AHRS_Library, strlen(striNEMO_AHRS_VERSION), (uint8_t*)striNEMO_AHRS_VERSION);
    
  }
}


/**
 * @brief  Get libraries version command handler.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoGetLibrariesHandler(uint8_t* pUsbFrame)
{
  uint8_t av_lib = AVAILABLE_LIBRARIES;

  if(pUsbFrame[0] & 0x20)
    iNemoSendAck(iNEMO_Get_Libraries, 1,&av_lib);

}

/**
 * @brief  Get the available sensors on the module. This frame comes from the sensors defines.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoGetAvailSensorsHandler(uint8_t* pUsbFrame)
{
  uint8_t cAvSensors=0;
  
#ifdef _6X
  cAvSensors |= (0x10 | 0x04);
#endif
  
#ifdef _GYRO
  cAvSensors |= 0x08;
#endif
  
#ifdef _PRESS
  cAvSensors |= 0x02;
#endif

#ifdef _TEMP
  cAvSensors |= 0x01;
#endif
    
  if(pUsbFrame[0] & 0x20)
    iNemoSendAck(iNEMO_Get_Available_Sensors, 1,&cAvSensors);
  
}

/**
 * @brief  Set output mode command handler.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoSetOutModeHandler(uint8_t* pUsbFrame)
{
  static const uint16_t nFreqEnum2freVal[] = {1,10,25,50,30,100,400};
  
  iNemoDataProcessClear();
  
  /* Always include the sensors samples into the data frame */
  iNemoDataProcessAppend(iNemoPackSensorData);
    
  /* Wich sensors */
  s_uOutSelect = pUsbFrame[3];

  /* Set the raw data flag */
  s_bRawDataEnabled = (pUsbFrame[3] & 0x20 ? TRUE : FALSE);
  
  /* Out mode flag (type of connection : USB...) --- can be removed !? */
  s_uOutMode = (pUsbFrame[4] & 0x07);

  /* Shift to compute the frequency index */
  s_uTimerFrequence = (pUsbFrame[4] & 0x38) >> 3;
  
#ifdef AHRS_MOD
  static FlagStatus s_xAhrsInited=RESET;
  
  /* AHRS request */
  if(pUsbFrame[3] & 0x80)
  {
    /* Init the AHRS parameters */
    if(!s_xAhrsInited)
    {
      /* Filter references for Acceleration and Magnetic field */
      s_xDataStruct.xSensorData.m_fAccRef[0]=0;
      s_xDataStruct.xSensorData.m_fAccRef[1]=0;
      s_xDataStruct.xSensorData.m_fAccRef[2]=-9.81f;
      
      s_xDataStruct.xSensorData.m_fMagRef[0]=0.37f;
      s_xDataStruct.xSensorData.m_fMagRef[1]=0;
      s_xDataStruct.xSensorData.m_fMagRef[2]=-0.25f;
      
      if(s_uTimerFrequence<7)
        s_xDataStruct.xSensorData.m_fDeltaTime=1.0f/nFreqEnum2freVal[s_uTimerFrequence];
      else
        s_xDataStruct.xSensorData.m_fDeltaTime=0.02f;
      
      s_xDataStruct.xSensorData.m_fVarAcc=5.346e-6;
      s_xDataStruct.xSensorData.m_fVarMag=5.346e-6;

      iNEMO_AHRS_Init(&s_xDataStruct.xSensorData, &s_xDataStruct.xEulerAngles, &s_xDataStruct.xQuat);
      s_xAhrsInited=SET;
    }
    
    iNemoDataProcessAppend(iNemoPackAHRSData);
  }
  else
  {
    /* DeInit the AHRS parameters */
    if(s_xAhrsInited)
    {
      iNEMO_AHRS_DeInit(&s_xDataStruct.xSensorData, &s_xDataStruct.xEulerAngles, &s_xDataStruct.xQuat);
      s_xAhrsInited=RESET;
    }
    
    
  }
#endif
  
  /* Compass request */
  if(pUsbFrame[3] & 0x40)
    iNemoDataProcessAppend(iNemoPackCompassData);
  
  iNemoDataProcessSetResponseFrameId(iNEMO_Start_Acquisition);

  /* If required set the ASK_DATA_MODE */
  if(pUsbFrame[4] & 0x40)
    s_bAskDataMode=TRUE;
    
  if((s_uTimerFrequence>7) || (s_uOutMode>0))
  {
    if(pUsbFrame[0] & 0x20)
      iNemoSendNack(iNEMO_SetOutMode,ValueOutOfRange);
  }
  else
  {
    if(s_uTimerFrequence!=7)
    {
      /* set timer only if s_uTimerFrequence is not 7 (in that case configure the sensor data ready, see the start frame handler) */
      iNemoTimerConfig((float)nFreqEnum2freVal[s_uTimerFrequence]);
    }
    
    /* set number of sample or continuos mode (s_nLengthSample=0) */
    s_nLengthSample = ((uint16_t)pUsbFrame[5] << 8) + pUsbFrame[6];
  
#ifdef _6X
    iNemoAccDataReadyConfig(DISABLE);
    iNemoMagDataReadyConfig(DISABLE);
#endif
#ifdef _GYRO    
    iNemoGyroDataReadyConfig(DISABLE);
#endif    
#ifdef _PRESS
    iNemoPressDataReadyConfig(DISABLE);
#endif
    
    if(pUsbFrame[0] & 0x20)
      iNemoSendAck(iNEMO_SetOutMode, 0, 0x00);
  }

}


/**
 * @brief  Get output mode command handler.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoGetOutModeHandler(uint8_t* pUsbFrame)
{
  uint8_t temp[4];
  temp[0]=s_uOutSelect;
  temp[1]=((s_uTimerFrequence ) << 3) + (s_uOutMode & 0x07);
  temp[2]=(uint8_t)(s_nLengthSample >> 8);
  temp[3]=(uint8_t)s_nLengthSample;
  
  if(pUsbFrame[0] & 0x20)
    iNemoSendAck(iNEMO_GetOutMode, 4, temp);
}


/**
 * @brief  Set sensor parameter command handler.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoSetSensorParameterHandler(uint8_t* pUsbFrame)
{

  if(iNemoSetSensor(&s_xDataStruct, pUsbFrame[3], pUsbFrame[4], (pUsbFrame[1] - 1), &pUsbFrame[5]))
  {
    if(pUsbFrame[0] & 0x20)
      iNemoSendAck(iNEMO_Set_Sensor_Parameter, 0, 0x00);
  }
  else
  {
    if(pUsbFrame[0] & 0x20)
      iNemoSendNack(iNEMO_Set_Sensor_Parameter,NotExecutable);
  }
}


/**
 * @brief  Get sensor parameter command handler.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoGetSensorParameterHandler(uint8_t* pUsbFrame)
{
  uint8_t value[10];

  if(iNemoGetSensorParam(&s_xDataStruct, pUsbFrame[3], pUsbFrame[4], value))
  {
    if(pUsbFrame[0] & 0x20)
      iNemoSendAck(iNEMO_Get_Sensor_Parameter, value[0], &value[1]);
  }
  else
  {
    if(pUsbFrame[0] & 0x20)
      iNemoSendNack(iNEMO_Get_Sensor_Parameter,NotExecutable);
  }
}


/**
 * @brief  Restore default sensor parameters command handler.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoRestoreDefaultParameterHandler(uint8_t* pUsbFrame)
{
  uint8_t value[10];

  if(iNemoRestoreDefaultParam(&s_xDataStruct, pUsbFrame[3], pUsbFrame[4], value))
  {
    if(pUsbFrame[0] & 0x20)
      iNemoSendAck(iNEMO_Restore_Default_Parameter,  value[0], &value[1]);
  }
  else
  {
    if(pUsbFrame[0] & 0x20)
      iNemoSendNack(iNEMO_Restore_Default_Parameter,NotExecutable);
  }
  
}


/**
 * @brief  Start acquisition command handler.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoStartAcquisitionHandler(uint8_t* pUsbFrame)
{
  s_iC=0;
  
  if(pUsbFrame[0] & 0x20)
    iNemoSendAck(iNEMO_Start_Acquisition, 0, 0x00);
  
  if(s_uTimerFrequence!=7)
    iNemoEnableTimer(ENABLE);
  else
  {
     switch(s_uOutSelect & 0x1F)
     {
     case 0x10:
#ifdef _6X
       iNemoAccDataReadyConfig(ENABLE);
#endif
       break;
     case 0x08:
#ifdef _GYRO
       iNemoGyroDataReadyConfig(ENABLE);
#endif
       break;
     case 0x04:
#ifdef _6X
       iNemoMagDataReadyConfig(ENABLE);
#endif
       break;
     case 0x02:
#ifdef _PRESS
       iNemoPressDataReadyConfig(ENABLE);
#endif
       break;
     }
  }
  
}


/**
 * @brief  Stop acquisition command handler.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoStopAcquisitionHandler(uint8_t* pUsbFrame)
{
  if(s_uTimerFrequence!=7)
  {
    iNemoEnableTimer(DISABLE);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
  else
  {
     switch(s_uOutSelect & 0x1F)
     {
     case 0x10:
#ifdef _6X
       iNemoAccDataReadyConfig(DISABLE);
#endif
       break;
     case 0x08:
#ifdef _GYRO
       iNemoGyroDataReadyConfig(DISABLE);
#endif
       break;
     case 0x04:
#ifdef _6X
       iNemoMagDataReadyConfig(DISABLE);
#endif
       break;
     case 0x02:
#ifdef _PRESS
       iNemoPressDataReadyConfig(DISABLE);
#endif
       break;
     }
  }
  
  s_iC=0;
  
  if(pUsbFrame[0] & 0x20)
    iNemoSendAck(iNEMO_Stop_Acquisition, 0, 0x00);

}


/**
 * @brief  Command handler to save the sensor parameters into flash.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoSaveToFlashHandler(uint8_t* pUsbFrame)
{
  iNemoSaveToFlash(&s_xDataStruct,SET);

  if(pUsbFrame[0] & 0x20)
    iNemoSendAck(iNEMO_Save_to_Flash, 0, 0x00);
 
}


/**
 * @brief  Command handler to load the sensor parameters from flash.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoLoadFromFlashHandler(uint8_t* pUsbFrame)
{
  iNemoReadFromFlash(&s_xDataStruct);
  
  if(pUsbFrame[0] & 0x20)
    iNemoSendAck(iNEMO_Load_from_Flash, 0, 0x00);
  
}


/**
 * @brief  Start HIC command handler.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoStartHICCalibrationHandler(uint8_t* pUsbFrame)
{
  s_iC=0;
  
  if(pUsbFrame[0] & 0x20)
    /* Send ack */
    iNemoSendAck(iNEMO_Start_HIC_Calibration, 0, 0x00);
  
  /* Clear and save the actual data process handler array */
  iNemoDataProcessClear();
  
  /* Append the HIC calibration handler */
  iNemoDataProcessAppend(iNemoPackHicCalibrationData);

#ifdef COMPASS_MOD
  /* Init the magnetic sensor HIC procedure */
  iNEMO_MagSensorCalibrationInit();
#endif
  
  /* Set <i>iNEMO_Start_HIC_Calibration</i> as the frame ID */
  iNemoDataProcessSetResponseFrameId(iNEMO_Start_HIC_Calibration);
  
  /* Start calibrating by starting the timer */
  iNemoEnableTimer(ENABLE);

}


/**
 * @brief  Stop HIC command handler.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoStopHICCalibrationHandler(uint8_t* pUsbFrame)
{
  /* Stop calibrating by stopping the timer */
  iNemoEnableTimer(DISABLE);
  
  /* Be sure the TIM IRQ is not masked */
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  
  /* Reset the time base */
  s_iC=0;
  
  /* Restore the old data processing handlers list */
  iNemoDataProcessRestore();
  
  /* Send ack */
  if(pUsbFrame[0] & 0x20)
    iNemoSendAck(iNEMO_Stop_HIC_Calibration, 0, 0x00);
  

}


/**
 * @brief  Sends the previously packed sensors data.
 * @param  pUsbFrame: USB frame received.
 * @retval None.
 */
void iNemoGetAcqDataHandler(uint8_t* pUsbFrame)
{
    /* call the function wich adds the frame header and pass it to the gatekeeper */
    iNemoSendData(s_xDataProcActStruct.cResponseFrameId, cNByteToSend, pDataBuffer);
}


/**
 * @}
 */


/**
 * @defgroup iNEMO_Handlers_Data_Process                Data Processing
 * @{
 */

/**
 * @brief Send a frame containing all sensor data and the algorithms data according to
 *        the transmission option specified previously.
 * @retval None.
 */
void iNemoDataProcess(void)
{   
  
  int16_t aux[2];
  
  /* this variable is static, cannot be reset during declaration */
  cNByteToSend=0;
 
  /* If continous acquisition is set or if the desired length has not been reached */
  if (s_nLengthSample == 0 || s_iC < s_nLengthSample)
  {
    /* Sample data from sensors */
    iNemoLibGetGyroRawData(s_xDataStruct.pnGyro);
    iNemoLibGetAccRawData(s_xDataStruct.pnAcc);
    iNemoLibGetMagRawData(s_xDataStruct.pnMag);
    s_xDataStruct.lPress=iNemoLibGetPressRawData();
    s_xDataStruct.nTemp=iNemoLibGetTempRawData();
    
    /* Align the gyro axis to the geomagnetic module one */
    aux[0] = -s_xDataStruct.pnGyro[1];
    aux[1] = s_xDataStruct.pnGyro[0];
    s_xDataStruct.pnGyro[0] = aux[0];
    s_xDataStruct.pnGyro[1] = aux[1];
    
    /* call the data process routines on the sampled data (they also build a byte buffer to be used as payload for the frame) */
    for(uint8_t i=0;i<s_xDataProcActStruct.cNDataProcActions;i++)
    {
      s_xDataProcActStruct.xDataProcessActions[i](pDataBuffer,&cNByteToSend); 
    }
    
    if(!s_bAskDataMode)
      /* call the function wich adds the frame header and pass it to the gatekeeper */
      iNemoSendData(s_xDataProcActStruct.cResponseFrameId, cNByteToSend, pDataBuffer);
  }
  else 
  {
    /* ..else disable the acquisition timer */
    iNemoEnableTimer(DISABLE);
  }
  
}


/**
 * @brief  Packs the sensor data according to the desired format.
 * @param  pcDataBuffer: pointer to the buffer where data have to be stored.
 *         This is a uint8_t array pointer.
 * @param  pcNByteToSend: pointer to the buffer size. The pointed value is modificed automatically by this routine.
 *         This is a uint8_t pointer.
 * @retval None.
 */
void iNemoPackSensorData(uint8_t *pcDataBuffer, uint8_t *pcNByteToSend)
{
    /* Timestamp increment */
    s_iC++;
    
    /* Timestamp packing */
    pcDataBuffer[0]=(uint8_t)(s_iC>>8);
    pcDataBuffer[1]=(uint8_t)(s_iC);
    
    /* 2 byte counter*/
    (*pcNByteToSend)+=2; 
    
    /* if accelerometer output is requested */
    if(s_uOutSelect & 0x10)
    {
      int16_t acc[3];
      
      for(uint8_t i=0;i<3;i++)
      {
        /*if raw data enabled*/
        if(s_bRawDataEnabled)
        {
          acc[i]=(int16_t)((s_xDataStruct.pnAcc[i] - (float)s_xDataStruct.nOffset[i]*s_xDataStruct.fSensitivity[i])/s_xDataStruct.fScaleFactor[i]);
        }
        /* Measurement unit data are requested*/
        else
        {
          acc[i]=(int16_t)(((float)s_xDataStruct.pnAcc[i]/s_xDataStruct.fSensitivity[i] - (float)s_xDataStruct.nOffset[i])/s_xDataStruct.fScaleFactor[i]);
        }
      }
      
      s16_to_u8_buffer(&acc[0], &pcDataBuffer[(*pcNByteToSend)]);
      s16_to_u8_buffer(&acc[1], &pcDataBuffer[(*pcNByteToSend) + 2]);
      s16_to_u8_buffer(&acc[2], &pcDataBuffer[(*pcNByteToSend) + 4]);
      
      (*pcNByteToSend)+=6;
      
    }/* End of Accelerometer Data*/

    /* if gyroscopes output is requested */
    if(s_uOutSelect & 0x08)
    {
      int16_t gyro[3];
      
      for(uint8_t i=0;i<3;i++)
      {
        /*if raw data enabled*/
        if(s_bRawDataEnabled)
        {
          gyro[i]=(int16_t)((float)(s_xDataStruct.pnGyro[i] - s_xDataStruct.nOffset[i+3]*s_xDataStruct.fSensitivity[i+3])/s_xDataStruct.fScaleFactor[i+3]);  
        }
        /* Measurement unit data are requested*/
        else
        {
          
          gyro[i]=(int16_t)(((float)(s_xDataStruct.pnGyro[i]/s_xDataStruct.fSensitivity[i+3]) - s_xDataStruct.nOffset[i+3])/s_xDataStruct.fScaleFactor[i+3]);  
        }
      }
      
      s16_to_u8_buffer(&gyro[0], &pcDataBuffer[(*pcNByteToSend)]);
      s16_to_u8_buffer(&gyro[1], &pcDataBuffer[(*pcNByteToSend) + 2]);
      s16_to_u8_buffer(&gyro[2], &pcDataBuffer[(*pcNByteToSend) + 4]);
      
      (*pcNByteToSend)+=6;
    }/* End of Gyroscope Data */

    /* if magnetometer output is requested */
    if(s_uOutSelect & 0x04)
    {
      int16_t mag[3];
      
      for(uint8_t i=0;i<3;i++)
      {
        /*if raw data enabled*/
        if(s_bRawDataEnabled)
        {
          mag[i]=(int16_t)((float)(s_xDataStruct.pnMag[i] - s_xDataStruct.nOffset[i+6]*s_xDataStruct.fSensitivity[i+6]/1000.0)/s_xDataStruct.fScaleFactor[i+6]);
          
        }
        /* Measurement unit data are requested*/
        else
        {  
          /* The register order is X, Z, Y*/
          mag[i]=(int16_t)(((float)s_xDataStruct.pnMag[i]*1000.0/s_xDataStruct.fSensitivity[i+6] - (float)s_xDataStruct.nOffset[i+6])/s_xDataStruct.fScaleFactor[i+6]); 
        } 
      }
      
      s16_to_u8_buffer(&mag[0], &pcDataBuffer[(*pcNByteToSend)]);
      s16_to_u8_buffer(&mag[1], &pcDataBuffer[(*pcNByteToSend) + 2]);
      s16_to_u8_buffer(&mag[2], &pcDataBuffer[(*pcNByteToSend) + 4]);
     
      (*pcNByteToSend)+=6;
    } /* End of Magnetometer Data */

    /* if pressure output is requested */
    if(s_uOutSelect & 0x02)
    {
      /*if raw data enabled*/
      if(s_bRawDataEnabled)
      {
        int32_t p;
        p= s_xDataStruct.lPress*100;
        s32_to_u8_buffer(&p, &pcDataBuffer[(*pcNByteToSend)]);
      }
      /* Measurement unit data are requested */
      else
      {    
        int32_t p;
        
        p=(int32_t)(((100.0*s_xDataStruct.lPress/s_xDataStruct.fSensitivity[9]) - s_xDataStruct.nOffset[9])/s_xDataStruct.fScaleFactor[9]);
        
        s32_to_u8_buffer(&p, &pcDataBuffer[(*pcNByteToSend)]);

      }
      
      (*pcNByteToSend)+=4;
    } /* End of Pressure Sensor Data */

    /* if temperature output is requested */
    if(s_uOutSelect & 0x01)
    {
      /*if raw data enabled*/
      if(s_bRawDataEnabled)
      {
        s16_to_u8_buffer(&s_xDataStruct.nTemp,&pcDataBuffer[(*pcNByteToSend)]);
      }
      /* Measurement unit data are requested*/
      else
      {
        int16_t p;
        
        p=(int16_t)((10.0*s_xDataStruct.nTemp/s_xDataStruct.fSensitivity[10] -10.0*( s_xDataStruct.nOffset[10]))/s_xDataStruct.fScaleFactor[10]);
        s16_to_u8_buffer(&p, &pcDataBuffer[(*pcNByteToSend)]);
      }
      
      (*pcNByteToSend)+=2;
    }  /* End of Temperature Sensor Data */
    
}


/**
 * @brief  Packs the AHRS data.
 * @param  pcDataBuffer: pointer to the buffer where data have to be stored.
 *         This is a uint8_t array pointer.
 * @param  pcNByteToSend: pointer to the buffer size. The pointed value is modificed automatically by this routine.
 *         This is a uint8_t pointer.
 * @retval None.
 */
void iNemoPackAHRSData(uint8_t *pcDataBuffer, uint8_t *pcNByteToSend)
{
#ifdef AHRS_MOD
  float roll, pitch, yaw;
  
  /* Accelerometer raw to measurements */
  s_xDataStruct.xSensorData.m_fAcc[0]=(float)s_xDataStruct.pnAcc[0]/s_xDataStruct.fSensitivity[0];
  s_xDataStruct.xSensorData.m_fAcc[1]=(float)s_xDataStruct.pnAcc[1]/s_xDataStruct.fSensitivity[1];
  s_xDataStruct.xSensorData.m_fAcc[2]=(float)s_xDataStruct.pnAcc[2]/s_xDataStruct.fSensitivity[2];
  
  /* Gyroscope Axis raw to measurements */
  s_xDataStruct.xSensorData.m_fGyro[0]=(float)s_xDataStruct.pnGyro[0]/s_xDataStruct.fSensitivity[3];
  s_xDataStruct.xSensorData.m_fGyro[1]=(float)s_xDataStruct.pnGyro[1]/s_xDataStruct.fSensitivity[4];
  s_xDataStruct.xSensorData.m_fGyro[2]=(float)s_xDataStruct.pnGyro[2]/s_xDataStruct.fSensitivity[5];
  
  /* Magnetometer Axis raw to measurements */
  s_xDataStruct.xSensorData.m_fMag[0]=(float)s_xDataStruct.pnMag[0]/s_xDataStruct.fSensitivity[6];
  s_xDataStruct.xSensorData.m_fMag[1]=(float)s_xDataStruct.pnMag[1]/s_xDataStruct.fSensitivity[7];
  s_xDataStruct.xSensorData.m_fMag[2]=(float)s_xDataStruct.pnMag[2]/s_xDataStruct.fSensitivity[8];
  
  
  
  for(uint8_t i=0;i<3;i++) 
  {  
    /* offset and gain applications */
    s_xDataStruct.xSensorData.m_fAcc[i]= (s_xDataStruct.xSensorData.m_fAcc[i] - s_xDataStruct.nOffset[i])/s_xDataStruct.fScaleFactor[i]*9.81f/1000.0f;
    s_xDataStruct.xSensorData.m_fGyro[i]= (s_xDataStruct.xSensorData.m_fGyro[i] - s_xDataStruct.nOffset[i+3])/s_xDataStruct.fScaleFactor[i+3]*3.141592f/180.0f;
    s_xDataStruct.xSensorData.m_fMag[i]= (s_xDataStruct.xSensorData.m_fMag[i] - s_xDataStruct.nOffset[i+6]/1000.0)/(s_xDataStruct.fScaleFactor[i+6]);
   
    if(i!=0)
    {
      /* axis alignments */
      s_xDataStruct.xSensorData.m_fAcc[i]=-s_xDataStruct.xSensorData.m_fAcc[i];
      s_xDataStruct.xSensorData.m_fGyro[i]=-s_xDataStruct.xSensorData.m_fGyro[i];
      s_xDataStruct.xSensorData.m_fMag[i]=-s_xDataStruct.xSensorData.m_fMag[i];
    }

  }
  
  iNEMO_AHRS_Update(&(s_xDataStruct.xSensorData), &(s_xDataStruct.xEulerAngles), &(s_xDataStruct.xQuat));
  roll  = s_xDataStruct.xEulerAngles.m_fRoll* 180.0f / 3.141592f;
  pitch = s_xDataStruct.xEulerAngles.m_fPitch * 180.0f / 3.141592f;
  yaw  = s_xDataStruct.xEulerAngles.m_fYaw * 180.0f / 3.141592f;
  Float_To_Buffer(roll, &pcDataBuffer[(*pcNByteToSend)]);
  Float_To_Buffer(pitch, &pcDataBuffer[(*pcNByteToSend) + 4]);
  Float_To_Buffer(yaw, &pcDataBuffer[(*pcNByteToSend) + 8]);
  Float_To_Buffer((s_xDataStruct.xQuat[0]), &pcDataBuffer[(*pcNByteToSend) + 12]);
  Float_To_Buffer((s_xDataStruct.xQuat[1]), &pcDataBuffer[(*pcNByteToSend) + 16]);
  Float_To_Buffer((s_xDataStruct.xQuat[2]), &pcDataBuffer[(*pcNByteToSend) + 20]);
  Float_To_Buffer((s_xDataStruct.xQuat[3]), &pcDataBuffer[(*pcNByteToSend) + 24]);
  (*pcNByteToSend) += 4*sizeof(float) + 3* sizeof(float);// 4*float quaternions + 3*float angles.
  
#endif


}


/**
 * @brief  Packs the compass data.
 * @param  pcDataBuffer: pointer to the buffer where data have to be stored.
 *         This is a uint8_t array pointer.
 * @param  pcNByteToSend: pointer to the buffer size. The pointed value is modificed automatically by this routine.
 *         This is a uint8_t pointer.
 * @retval None.
 */
void iNemoPackCompassData(uint8_t *pcDataBuffer, uint8_t *pcNByteToSend)
{
#ifdef COMPASS_MOD
  float pfRPH[3];
  
  /* Accelerometer raw to measurements */
  s_xDataStruct.xSensorData.m_fAcc[0]=((float)(s_xDataStruct.pnAcc[0])/s_xDataStruct.fSensitivity[0])*9.8f/1000.0f;
  s_xDataStruct.xSensorData.m_fAcc[1]=((float)(s_xDataStruct.pnAcc[1])/s_xDataStruct.fSensitivity[1])*9.8f/1000.0f;
  s_xDataStruct.xSensorData.m_fAcc[2]=((float)(s_xDataStruct.pnAcc[2])/s_xDataStruct.fSensitivity[2])*9.8f/1000.0f;
  
  /* Magnetometer raw to measurements */
  s_xDataStruct.xSensorData.m_fMag[0]=((float)s_xDataStruct.pnMag[0]*1000.0)/s_xDataStruct.fSensitivity[6];
  s_xDataStruct.xSensorData.m_fMag[1]=((float)s_xDataStruct.pnMag[1]*1000.0)/s_xDataStruct.fSensitivity[7];
  s_xDataStruct.xSensorData.m_fMag[2]=((float)s_xDataStruct.pnMag[2]*1000.0)/s_xDataStruct.fSensitivity[8];
  
  for(uint8_t i=0;i<3;i++) 
  {  
    /* offset and gain applications */
    s_xDataStruct.xSensorData.m_fAcc[i]= (s_xDataStruct.xSensorData.m_fAcc[i] - s_xDataStruct.nOffset[i])/s_xDataStruct.fScaleFactor[i];
    s_xDataStruct.xSensorData.m_fMag[i]= (s_xDataStruct.xSensorData.m_fMag[i] - s_xDataStruct.nOffset[i+6])/s_xDataStruct.fScaleFactor[i+6]; 
  }
  
  /* axis alignments */
  s_xDataStruct.xSensorData.m_fAcc[0]=-s_xDataStruct.xSensorData.m_fAcc[0];
  s_xDataStruct.xSensorData.m_fMag[1]=-s_xDataStruct.xSensorData.m_fMag[1];
  s_xDataStruct.xSensorData.m_fMag[2]=-s_xDataStruct.xSensorData.m_fMag[2];
  
  /* Tilted compass update data output */
  iNEMO_TiltedCompass(s_xDataStruct.xSensorData.m_fMag, s_xDataStruct.xSensorData.m_fAcc, pfRPH);
  
  /* Store the computed heading into the s_xDataStruct */
  s_xDataStruct.fHeading = pfRPH[2];
  
  for(uint8_t i=0;i<3;i++)
    pfRPH[i]  = pfRPH[i]*180.0f/3.141592f;
  
  Float_To_Buffer(pfRPH[0], &pcDataBuffer[(*pcNByteToSend)]);
  Float_To_Buffer(pfRPH[1], &pcDataBuffer[(*pcNByteToSend) + 4]);
  Float_To_Buffer(pfRPH[2], &pcDataBuffer[(*pcNByteToSend) + 8]);
  
  (*pcNByteToSend) += 3*sizeof(float); // 3*float angles.
#endif      
  
}


/**
 * @brief  Packs the HIC data. The returned value is a calibration progress extimation.
 * @param  pcDataBuffer: pointer to the buffer where data have to be stored.
 *         This is a uint8_t array pointer.
 * @param  pcNByteToSend: pointer to the buffer size. The pointed value is modificed automatically by this routine.
 *         This is a uint8_t pointer.
 * @retval None.
 */
void iNemoPackHicCalibrationData(uint8_t *pcDataBuffer, uint8_t *pcNByteToSend)
{
#ifdef COMPASS_MOD
  uint8_t cMagCalibProgress;
  float pfGain[3],pfOffset[3],fGainMean;
  
  /* The register order is X, Z, Y */
  s_xDataStruct.xSensorData.m_fMag[0]=((float)s_xDataStruct.pnMag[0]*1000.0)/s_xDataStruct.fSensitivity[6];
  s_xDataStruct.xSensorData.m_fMag[1]=((float)s_xDataStruct.pnMag[1]*1000.0)/s_xDataStruct.fSensitivity[7];
  s_xDataStruct.xSensorData.m_fMag[2]=((float)s_xDataStruct.pnMag[2]*1000.0)/s_xDataStruct.fSensitivity[8];


  /* calibration loop body */
  cMagCalibProgress=iNEMO_MagSensorCalibrationRun(s_xDataStruct.xSensorData.m_fMag,pfGain,pfOffset);


  /* Compute the gain mean (an extimation of the EMF) in order to normalize between -EMF and EMF */
  fGainMean=(pfGain[0]+pfGain[1]+pfGain[2])/3;
  
  if(fGainMean==0.0)
    fGainMean=1.0;
  
  /* save gains and offsets */
  for(uint8_t i=0;i<3;i++)
  {
    s_xDataStruct.fScaleFactor[6+i] = pfGain[i]/fGainMean;
    s_xDataStruct.nOffset[6+i] = (int16_t) pfOffset[i];
  }
  
  /* pack the frame with the progress value */      
  pcDataBuffer[(*pcNByteToSend)]=cMagCalibProgress;
  (*pcNByteToSend)++;
  
#endif
}

/**
 * @}
 */


/**
 * @defgroup iNEMO_Handlers_Utils_Functions                Utilities
 * @{
 */

/**
 * @brief  Data structure init function.
 * @retval None.
 */
void iNemoInitData(void)
{
  /* Init the data structure */
  iNemoSensorParametersInit(&s_xDataStruct);

  iNemoDataProcessClear();
  iNemoDataProcessAppend(iNemoPackSensorData);
   
#if defined(AHRS_MOD) && defined(AHRS_JUMP)
   /* This is just to make sure the compiler allocates the array */
   dummyFill4Ahrs[0]=0;
    
   /* Initialize the AHRS function pointers */
   AHRS_POINTERS_INIT(iNEMO_AHRS_Init,iNEMO_AHRS_DeInit,iNEMO_AHRS_Update);
#endif
}


/**
 * @brief  Returns the connection state.
 * @retval bool connection state.
 *         This parameter can be TRUE or FALSE.
 */
bool iNemoGetConnectionState(void)
{
  return s_bConnectState;

}


/**
 * @brief Sends a generic frame.
 * @param cFrameId: Frame code to ack.
 * @param cPayloadLength: payload length.<br><b>NOTE</b> This parameter is the payload 
 *        length intended as the length of the buffer pointed by pcPayload (not the whole frame length).
 * @param pcPayload: pointer to the buffer to be used as payload (if any).
 * @param cCtrlType: control field.
 * @note It is recommended to not use this function directly but one of the iNEMO_GUI_Macros in its charge.
 * @retval None.
 */
void iNemoSendFrame(uint8_t cFrameId, uint8_t cPayloadLength, uint8_t* pcPayload, uint8_t cCtrlType)
{
    /* The buffer related to the data frame */
    static uint8_t ucDataFrame[MAX_FRAME_DIM];
    
    /* The MoreFragment length */
    uint16_t ucMf = 0;
    /* The LastFragment length */
    uint8_t ucLf = 0;
    /* The MoreFragment index */
    uint16_t ucMfIndex = 0;
    
    /* Calculate the MoreFragment length */
    ucMf = cPayloadLength/(MAX_FRAME_DIM-3);
    
    /* Calculate the LastFragment length */
    ucLf = cPayloadLength%(MAX_FRAME_DIM-3);
    
    /* As long as there are still MoreFragment frame to send*/
    while(ucMf--)
    {
      /* Build the header of the frame */
      ucDataFrame[0] = cCtrlType | 0x10;
      ucDataFrame[1] = (MAX_FRAME_DIM-2);
      ucDataFrame[2] = cFrameId;
      
      /* The payload field of the data frame */
      for(uint8_t ucI=0; ucI<(MAX_FRAME_DIM-3); ucI++)
        ucDataFrame[ucI+3] = pcPayload[ucI+ucMfIndex*(MAX_FRAME_DIM-3)];
      
      /* Send the frame via usb */
      iNemoGKSendData(ucDataFrame, MAX_FRAME_DIM);
      
      /* Increase the MoreFragment index */
      ucMfIndex++;
    }
    
    /* Build the header of the frame */
    ucDataFrame[0] = cCtrlType;
    ucDataFrame[1] = ucLf+1;
    ucDataFrame[2] = cFrameId;
    
    /* The payload field of the data frame */
    for(uint8_t ucI=0; ucI<ucLf; ucI++)
      ucDataFrame[ucI+3] = pcPayload[ucI+ucMfIndex*(MAX_FRAME_DIM-3)];
    
    /* Send the frame via usb */
    iNemoGKSendData(ucDataFrame, ucDataFrame[1]+2);
  
}



/**
 * @brief Start and stop the timer used by the iNemo Data Task.
 * @param command start the timer if ENABLE, stop the timer if DISABLE.
 * @retval None.
 */
void iNemoEnableTimer(FunctionalState command) {
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  TIM_ITConfig( TIM2, TIM_IT_Update, command );
  TIM_Cmd(TIM2, command);
  
}


/**
 * @brief return a boolean to check if Trace is enabled/disabled.
 * @retval s_bAhrsEnabled : TRUE trace enable, FALSE trace disable.
 */
bool iNemoGetTrace(void)
{
  return s_bTraceEnabled;
}


/**
 * @brief set a boolean to enable/disable  TRACE
 * @param bEnable :  TRUE trace enable, FALSE trace disable
 * @retval None.
 */
void iNemoSetTrace(bool bEnable)
{
  s_bTraceEnabled = bEnable;
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

/**
 * @}
 */

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
