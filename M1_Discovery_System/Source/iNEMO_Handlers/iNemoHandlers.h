/**
*
* @file     iNemoHandlers.h
* @author   IMS Systems Lab - ART Team
* @version  V2.3.0
* @date    01 February 2013
* @brief    Entry point for GUI layer and relative API
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
*
*/

/**
 *  Define to prevent recursive inclusion
*/
#ifndef __GUI_LAYER_H
#define __GUI_LAYER_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f10x.h"
#include "iNemoLib.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "iNemoGatekeeper.h"

   
/**
 * @addtogroup iNEMO_Discovery_System
 * @{
 */

/**
 * @addtogroup iNEMO_Handlers
 * @{
 */

/**
 * @addtogroup iNEMO_Handlers_Macros                iNemo Handlers Macros
 * @brief These macros are used as high level communications commands.
 * @{
 */

//#define BOARD_ID    2
   
/**
 * @brief Sends a DATA frame by calling the iNemoSendFrame with the right parameters.
 * @param cFrameId: Frame code.
 * @param cPayloadLength: payload length.<br><b>NOTE</b> This parameter is the payload 
 *        length intended as the length of the buffer pointed by pcPayload (not the whole frame length).
 * @param pcPayload: pointer to the buffer to be used as payload.
 * @retval None.
 */
#define iNemoSendData(cFrameId, cPayloadLength, pcPayload)        iNemoSendFrame(cFrameId, cPayloadLength, pcPayload, DATA_TYPE)   


/**
 * @brief Sends an ACK frame by calling the iNemoSendFrame with the right parameters.
 * @param cFrameId: Frame code to ack.
 * @param cPayloadLength: payload length.<br><b>NOTE</b> This parameter is the payload 
 *        length intended as the length of the buffer pointed by pcPayload (not the whole frame length).
 * @param pcPayload: pointer to the buffer to be used as payload (if any).
 * @retval None.
 */
#define iNemoSendAck(cFrameId, cPayloadLength, pcPayload)         iNemoSendFrame(cFrameId, cPayloadLength, pcPayload, ACK_TYPE)   


/**
 * @brief Sends a NACK frame by calling the iNemoSendFrame with the right parameters.
 * @param cFrameId: Frame code to Nack.
 * @param cErrorcode: error code for Nack. This parameter can be one among ERROR_CODE .
 * @retval None.
 */
#define iNemoSendNack(cFrameId, cErrorcode)                {uint8_t payload=cErrorcode; iNemoSendFrame(cFrameId, 1, &payload, NACK_TYPE);}   


/**
 * @brief Sends a TRACE frame by calling the iNemoSendFrame with the right parameters.
 * @param cFrameId : Frame code to Nack.
 * @param cPayloadLength: payload length.<br><b>NOTE</b> This parameter is the payload 
 *        length intended as the length of the buffer pointed by pcPayload (not the whole frame length).
 * @param pcPayload: pointer to the buffer to be used as payload.
 * @retval None.
 */
#define iNemoSendTrace(cFrameId, cPayloadLength, pcPayload)       iNemoSendFrame(cFrameId, cPayloadLength, pcPayload, TRACE_TYPE)   


/**
* @}
*/

/**
 * @addtogroup iNEMO_Handlers_Constants                iNemo Handlers Constants
 * @brief Any constant used to manage the communication and the commands.
 * @{
 */

/**
* @addtogroup FRAME_CONTROL_DEFINE              Frame Control
* @brief Defines and macros to manage the frame control field.
* @{
*/

/**
 * @brief  Control frame type.
 */ 
#define CTRL_type            0x00 
/**
 * @brief  Data frame type.
 */ 
#define DATA_type            0x40 
/**
 * @brief  Ack frame type.
 */ 
#define ACK_type             0x80 
/**
 * @brief  Nack frame type.
 */ 
#define NACK_type            0xC0 

/**
 * @brief  Ack request enabling mask.
 */ 
#define ACK_req              0x20 
/**
 * @brief  Ack request disabling mask.
 */ 
#define ACK_NOTreq           0x00 

/**
 * @brief  Last Fragment mask.
 */ 
#define Last_Frag            0x00 
/**
 * @brief  More Fragment mask.
 */ 
#define More_Frag            0x10 

/**
 * @brief  Protocol Version.
 */ 
#define Version_1            0x00 

/**
 * @brief  Quality of Service Normal mask.
 */
#define QoS_Normal           0x00 
/**
 * @brief  Quality of Service Medium mask.
 */
#define Qos_Medium           0x01 
/**
 * @brief  Quality of Service High mask.
 */
#define QoS_High             0x02 

 
/**
 * @brief  Macro to build control byte.
 */   
#define CTRL_FIELD_BUILD(type,ack,frag,vers,QoS) (type | ack | frag | vers | QoS) 
/**
 * @brief  Macro to build control frame with ack, last fragment.
 */
#define CTRL_wACK_LF         CTRL_FIELD_BUILD(CTRL_type, ACK_req ,Last_Frag ,Version_1 ,QoS_Normal)  
/**
 * @brief  Macro to build control frame without ack,last fragment
 */
#define CTRL_noACK_LF        CTRL_FIELD_BUILD(CTRL_type, ACK_NOTreq ,Last_Frag ,Version_1 ,QoS_Normal) 
/**
 * @brief  Macro to build data frame without ack, last fragment
 */
#define DATA_TYPE            CTRL_FIELD_BUILD(DATA_type, ACK_NOTreq ,Last_Frag ,Version_1 ,QoS_Normal) 
/**
 * @brief  Macro to build Ack frame
 */
#define ACK_TYPE             CTRL_FIELD_BUILD(ACK_type, ACK_NOTreq, Last_Frag ,Version_1 ,QoS_Normal) 
/**
 * @brief  Macro to build NACK frame
 */
#define NACK_TYPE            CTRL_FIELD_BUILD(NACK_type, ACK_NOTreq ,Last_Frag ,Version_1 ,QoS_Normal) 
/**
 * @brief  Macro to build data frame without ack, last fragment, QOS Medium
 */
#define TRACE_TYPE           CTRL_FIELD_BUILD(DATA_type, ACK_NOTreq ,Last_Frag ,Version_1 ,Qos_Medium) 



/**
* @}
*/



/**
* @addtogroup ERROR_CODE                      Error codes
* @brief The error codes that can be returned by a NACK frame.
* @{
*/
/**
 * @brief  Error code for Unsupported Command
 */
#define  CmdUnsupported     0x01
/**
 * @brief  Error code for Value Out of Range
 */
#define  ValueOutOfRange    0x02
/**
 * @brief  Error code for Command not executable
 */
#define  NotExecutable      0x03
/**
 * @brief  Error code for Wrong Syntax
 */
#define  WrongSyntax        0x04
/**
 * @brief  Error code because in Not Connected State
 */
#define  NotConnected       0x05

/**
* @}
*/


/**
 * @addtogroup MESSAGE_ID                        Message IDs
 * @brief Frame message ID list.
 * @{
 */

/**
 * @addtogroup COMMUNICATION_CONTROL_FRAME       Communication control
 * @brief Frames to control the communication.
 * @{
 */

/**
 * @brief  iNEMO_Connect frame code
 */
#define iNEMO_Connect                 0x00
/**
 * @brief  iNEMO_Connect frame length
 */
#define iNEMO_Connect_Length          1

/**
 * @brief  iNEMO_Disconnect frame code
 */
#define iNEMO_Disconnect              0x01
/**
 * @brief  iNEMO_Disconnect frame length
 */
#define iNEMO_Disconnect_Length       1

/**
 * @brief  iNEMO_Reset_Board frame code
 */
#define iNEMO_Reset_Board             0x02
/**
 * @brief  iNEMO_Reset_Board frame length
 */
#define iNEMO_Reset_Board_Length      1

/**
 * @brief  iNEMO_Enter_DFU_Mode frame code
 */
#define iNEMO_Enter_DFU_Mode          0x03
/**
 * @brief  iNEMO_Enter_DFU_Mode frame length
 */
#define iNEMO_Enter_DFU_Mode_Length   1

/**
 * @brief  iNEMO_Trace frame code
 */
#define iNEMO_Trace                   0x07
/**
 * @brief  iNEMO_Trace frame length
 */
#define iNEMO_Trace_Length            2

/**
 * @brief  iNEMO_Led frame code
 */
#define iNEMO_Led                     0x08
/**
 * @brief  iNEMO_Led frame length
 */
#define iNEMO_Led_Length              2   


/**
* @}
*/

/**
 * @addtogroup BOARD_INFO_FRAME                   Board info
 * @brief Frames to get some board properties.
 * @{
 */

/**
 * @brief  iNEMO_Get_Device_Mode frame code
 */
#define iNEMO_Get_Device_Mode         0x10
/**
 * @brief  iNEMO_Get_Device_Mode frame length
 */
#define iNEMO_Get_Device_Mode_Length  1   

/**
 * @brief  iNEMO_Get_MCU_ID frame code
 */   
#define iNEMO_Get_MCU_ID              0x12
/**
 * @brief  iNEMO_Get_MCU_ID frame length
 */
#define iNEMO_Get_MCU_ID_Length       1  

/**
 * @brief  iNEMO_Get_FW_Version frame code
 */
#define iNEMO_Get_FW_Version          0x13
/**
 * @brief  iNEMO_Get_FW_Version frame length
 */
#define iNEMO_Get_FW_Version_Length   1   

/**
 * @brief  iNEMO_Get_HW_Version frame code
 */   
#define iNEMO_Get_HW_Version          0x14
/**
 * @brief  iNEMO_Get_HW_Version frame length
 */
#define iNEMO_Get_HW_Version_Length   1   

/**
 * @brief  iNEMO_Identify frame code
 */
#define iNEMO_Identify                0x15
/**
 * @brief  iNEMO_Identify frame length
 */
#define iNEMO_Identify_Length         1


/**
 * @brief  iNEMO_Get_AHRS_Library frame code
 */   
#define iNEMO_Get_AHRS_Library        0x17
/**
 * @brief  iNEMO_Get_AHRS_Library frame length
 */
#define iNEMO_Get_AHRS_Library_Length 1

/**
 * @brief  iNEMO_Get_Libraries frame code
 */
#define iNEMO_Get_Libraries           0x18
/**
 * @brief  iNEMO_Get_Libraries frame length
 */
#define iNEMO_Get_Libraries_Length    1

/**
 * @brief  iNEMO_Get_Available_Sensors frame code
 */
#define iNEMO_Get_Available_Sensors           0x19
/**
 * @brief  iNEMO_Get_Available_Sensors frame length
 */
#define iNEMO_Get_Available_Sensors_Length    1   
   
   
/**
* @}
*/

/**
 * @addtogroup SENSOR_SETTING_FRAME                Sensors setting
 * @brief Frames to manage the operations on the sensor parameters.
 * @{
 */

/**
 * @brief  iNEMO_Set_Sensor_Parameter frame code
 */
#define iNEMO_Set_Sensor_Parameter             0x20
/**
 * @brief  iNEMO_Set_Sensor_Parameter frame length
 */
#define iNEMO_Set_Sensor_Parameter_Length      0   

/**
 * @brief  iNEMO_Get_Sensor_Parameter frame code
 */   
#define iNEMO_Get_Sensor_Parameter             0x21
/**
 * @brief  iNEMO_Get_Sensor_Parameter frame length
 */
#define iNEMO_Get_Sensor_Parameter_Length      3

/**
 * @brief  iNEMO_Restore_Default_Parameter frame code
 */ 
#define iNEMO_Restore_Default_Parameter        0x22
/**
 * @brief  iNEMO_Restore_Default_Parameter frame length
 */
#define iNEMO_Restore_Default_Parameter_Length 3   

/**
 * @brief  iNEMO_Save_to_Flash frame code
 */ 
#define iNEMO_Save_to_Flash                    0x23
/**
 * @brief  iNEMO_Save_to_Flash frame length
 */
#define iNEMO_Save_to_Flash_Length             1

/**
 * @brief  iNEMO_Load_from_Flash frame code
 */
#define iNEMO_Load_from_Flash                  0x24
/**
 * @brief  iNEMO_Load_from_Flash frame length
 */
#define iNEMO_Load_from_Flash_Length           1
   
/**
 * @}
 */


/**
 * @addtogroup ACQUISITION_SENSOR_DATA_FRAME     Acquisition Sensor Data
 * @brief Frames dealing with the acquisition of data.
 * @{
 */

/**
 * @brief  iNEMO_SetOutMode frame code
 */
#define iNEMO_SetOutMode                      0x50
/**
 * @brief  iNEMO_Load_from_Flash frame length
 */
#define iNEMO_SetOutMode_Length               5

/**
 * @brief  iNEMO_GetOutMode frame code
 */   
#define iNEMO_GetOutMode                      0x51
/**
 * @brief  iNEMO_GetOutMode frame length
 */
#define iNEMO_GetOutMode_Length               1   

/**
 * @brief  iNEMO_Start_Acquisition frame code
 */
#define iNEMO_Start_Acquisition               0x52
/**
 * @brief  iNEMO_Start_Acquisition frame length
 */
#define iNEMO_Start_Acquisition_Length        1

/**
 * @brief  iNEMO_Stop_Acquisition frame code
 */
#define iNEMO_Stop_Acquisition                0x53
/**
 * @brief  iNEMO_Stop_Acquisition frame length
 */
#define iNEMO_Stop_Acquisition_Length         1   

/**
 * @brief  iNEMO_Get_Acq_Data frame code
 */
#define iNEMO_Get_Acq_Data                    0x54
/**
 * @brief  iNEMO_Get_Acq_Data frame length
 */
#define iNEMO_Get_Acq_Data_Length             1   

/**
 * @brief  iNEMO_Start_HIC_Calibration frame code
 */
#define iNEMO_Start_HIC_Calibration           0x60
/**
 * @brief  iNEMO_Start_HIC_Calibration frame length
 */
#define iNEMO_Start_HIC_Calibration_Length    1   

/**
 * @brief  iNEMO_Stop_HIC_Calibration frame code
 */
#define iNEMO_Stop_HIC_Calibration            0x61
/**
 * @brief  iNEMO_Stop_HIC_Calibration frame length
 */
#define iNEMO_Stop_HIC_Calibration_Length     1 

/**
 * @}
 */


/**
 * @}
 */



/**
 * @addtogroup GENERIC_CONSTANTS                  Generic Constants
 * @brief Some generic const.
 * @{
 */

/**
 * @brief  1 HZ frequency acquisition code
 */
#define LOW_FREQUENCY         0x00    
/**
 * @brief  10 HZ frequency acquisition code
 */
#define MEDIUM_FREQUENCY_1    0x01    
/**
 * @brief  25 HZ frequency acquisition code
 */
#define MEDIUM_FREQUENCY_2    0x02    
/**
 * @brief  50 HZ frequency acquisition code
 */
#define HIGH_FREQUENCY        0x03    


/**
 * @addtogroup FW_HW_VERSION                         Fw and Hw version
 * @brief Hardware and firmware version.
 * @{
 */
/**
 * @brief  Firmware Version string
 */
#define iNEMO_FIRMWARE_VERSION	    "iNEMO Firmware_Version_2.5.0"
/**
 * @brief  Size of Firmware Version string
 */
#define SIZE_FWversion  strlen(iNEMO_FIRMWARE_VERSION)

/**
 * @brief  Hardware Version string
 */
#define iNEMO_HARDWARE_VERSION	    "iNEMO M1 Hardware_Version_1"
/**
 * @brief  Size of Hardware Version string
 */
#define SIZE_HWversion  strlen(iNEMO_HARDWARE_VERSION)

/**
 * @}
 */


/**
 * @addtogroup LIBS_VERSION                   Libraries version
 * @brief Available libraries and functions.
 * @{
 */
#ifdef AHRS_MOD
/* Don't define anything here because the AHRS version string is imported from the AHRS header file */
/**
 * @brief  AHRS Library version
 */
#define iNEMO_AHRS_LIBRARY              0x01
#else
#define iNEMO_AHRS_LIBRARY	   	0x00
#define iNEMO_AHRS_VERSION_N		0
#define iNEMO_AHRS_VERSION_SUBN		0
#define iNEMO_AHRS_VERSION_SUBMIN	0
#endif
   

#ifdef COMPASS_MOD
/**
 * @brief  Compass Library version case Compass is enabled
 */
#define iNEMO_COMPASS_LIBRARY        0x02
#else
/**
 * @brief  Compass Library version case Compass is not enabled
 */
#define iNEMO_COMPASS_LIBRARY	     0x00
#endif

#ifdef ALTIMETER_MOD
/**
 * @brief  Altimiter Library version case altimiter is enabled
 */
#define iNEMO_ALTIMETER_LIBRARY      0x04
#else
/**
 * @brief  Altimiter Library version case altimiter is not enabled
 */
#define iNEMO_ALTIMETER_LIBRARY	     0x00
#endif

#ifdef TRACE_MOD
/**
 * @brief  Trace Library version case trace is enabled
 */
#define iNEMO_TRACE_LIBRARY          0x08
#else
/**
 * @brief  Trace Library version case trace is not enabled
 */
#define iNEMO_TRACE_LIBRARY	     0x00
#endif

#ifdef FAT_MOD
/**
 * @brief  FAT Library version case trace is enabled
 */
#define iNEMO_FAT_LIBRARY            0x10
#else
/**
 * @brief  FAT Library version case trace is not enabled
 */
#define iNEMO_FAT_LIBRARY	     0x00
#endif

/**
 * @brief  Macro to build the available libraries
 */
#define AVAILABLE_LIBRARIES     (iNEMO_AHRS_LIBRARY | iNEMO_COMPASS_LIBRARY | \
                                  iNEMO_ALTIMETER_LIBRARY | iNEMO_TRACE_LIBRARY |\
                                  iNEMO_FAT_LIBRARY) 



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
 * @addtogroup iNEMO_Handlers_Private_Variables                      iNemo Handlers Private Variables
 * @brief The gate keeper queue used to receive frames.
 * @{
 */  
/**
 * @brief  Variable for Queue
 */
extern xQueueHandle xUsbGK2ParserQueue;

/**
 * @}
 */

/**
 * @addtogroup iNEMO_Handlers_Functions                         iNemo Handlers Functions
 * @brief iNemo Handlers Functions.
 * @{
 */


/**
 * @defgroup iNEMO_Handlers_Data_Process                Data Processing
 * @brief Data processing. These functions make some maths on the @ref s_xDataStruct
 *        and then pack the result in a buffer to be sent as a data frame payload.
 * @{
 */
void iNemoDataProcess(void);
void iNemoPackSensorData(uint8_t *pcDataBuffer, uint8_t *pcNByteToSend);
void iNemoPackAHRSData(uint8_t *pcDataBuffer, uint8_t *pcNByteToSend);
void iNemoPackCompassData(uint8_t *pcDataBuffer, uint8_t *pcNByteToSend);
void iNemoPackHicCalibrationData(uint8_t *pcDataBuffer, uint8_t *pcNByteToSend);

/**
 * @}
 */


/**
 * @defgroup iNEMO_Handlers_Handlers                Frame Command Handlers
 * @brief Command Handlers. These functions are called directly from @ref iNemoCommandTaskFunction.
 *        Their pointers are assigned to the @ref s_xiNemoFrameActionMap array to be quickly called when a 
 *        matching ID frame arrives.
 * @{
 */

void iNemoSendFrame(uint8_t ucFrameId, uint8_t ucFrameLength, uint8_t* ucPayload, uint8_t cCtrlType);
void iNemoConnectHandler(uint8_t* pUsbFrame);
void iNemoDisconnectHandler(uint8_t* pUsbFrame);
void iNemoResetBoardHandler(uint8_t* pUsbFrame);
void iNemoEnterDfuModeHandler(uint8_t* pUsbFrame);
void iNemoLedHandler(uint8_t* pUsbFrame);
void iNemoGetDeviceModeHandler(uint8_t* pUsbFrame);
void iNemoGetMcuIdHandler(uint8_t* pUsbFrame);
void iNemoGetFwVersionHandler(uint8_t* pUsbFrame);
void iNemoGetHwVersionHandler(uint8_t* pUsbFrame);
void iNemoIdentifyHandler(uint8_t* pUsbFrame);
void iNemoGetAhrsLibraryHandler(uint8_t* pUsbFrame);
void iNemoGetLibrariesHandler(uint8_t* pUsbFrame);
void iNemoGetAvailSensorsHandler(uint8_t* pUsbFrame);
void iNemoSetOutModeHandler(uint8_t* pUsbFrame);
void iNemoGetOutModeHandler(uint8_t* pUsbFrame);
void iNemoSetSensorParameterHandler(uint8_t* pUsbFrame);
void iNemoGetSensorParameterHandler(uint8_t* pUsbFrame);
void iNemoRestoreDefaultParameterHandler(uint8_t* pUsbFrame);
void iNemoStartAcquisitionHandler(uint8_t* pUsbFrame);
void iNemoStopAcquisitionHandler(uint8_t* pUsbFrame);
void iNemoStartHICCalibrationHandler(uint8_t* pUsbFrame);
void iNemoStopHICCalibrationHandler(uint8_t* pUsbFrame);
void iNemoSaveToFlashHandler(uint8_t* pUsbFrame);
void iNemoLoadFromFlashHandler(uint8_t* pUsbFrame);
void iNemoGetAcqDataHandler(uint8_t* pUsbFrame);

/**
 * @}
 */

/**
 * @defgroup iNEMO_Handlers_Utils_Functions                Utilities
 * @brief Some generic functions.
 * @{
 */
void iNemoInitData(void);
bool iNemoGetConnectionState(void);
void iNemoSetTrace(bool bEnable);
bool iNemoGetTrace(void);

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

#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
