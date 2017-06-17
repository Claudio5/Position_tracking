/**
 * @file     STM32F1_VC_Prop.c
 * @author   MSH RF/ART Team IMS-Systems Lab
 * @version  V1.0.0
 * @date     August 4, 2011
 * @brief    Insert here the description.
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
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "STM32F1_VC_Prop.h"
#include "STM32F1_VC_General.h"
#include "STM32F1_VC_Pwr.h"


/** @addtogroup STM32F1_Virtual_Com
 * @{
 */


/** @addtogroup STM32F1_VC_Properties
 * @{
 */


/** @defgroup STM32F1_VC_Properties_Private_TypesDefinitions   STM32F1 VC Properties Private TypesDefinitions
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_Properties_Private_Defines    STM32F1 VC Properties Private Defines
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_Properties_Private_Macros   STM32F1 VC Properties Private Macros
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_Properties_Private_Variables    STM32F1 VC Properties Private Variables
 * @{
 */
/**
 * @brief Request
 */
uint8_t Request = 0;

/**
 * @brief USB line coding structure
 */
LineCoding linecoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* no. of bits 8*/
  };

/**
 * @brief Device Table structure
 */
DEVICE Device_Table =
  {
    EP_NUM,
    1
  };

/**
 * @brief Device Propoerty structure
 */
DEVICE_PROP Device_Property =
  {
    Stm32f1VCPortInit,
    Stm32f1VCPortReset,
    Stm32f1VCPortStatusIn,
    Stm32f1VCPortStatusOut,
    Stm32f1VCPortDataSetup,
    Stm32f1VCPortNoDataSetup,
    Stm32f1VCPortGetInterfaceSetting,
    Stm32f1VCPortGetDeviceDescriptor,
    Stm32f1VCPortGetConfigDescriptor,
    Stm32f1VCPortGetStringDescriptor,
    0,
    0x40 /*MAX PACKET SIZE*/
  };

/**
 * @brief User Standard Requests structure
 */
USER_STANDARD_REQUESTS User_Standard_Requests =
  {
    Stm32f1VCPortGetConfiguration,
    Stm32f1VCPortSetConfiguration,
    SdkEvalVCPortGetInterface,
    SdkEvalVCPortSetInterface,
    Stm32f1VCPortGetStatus,
    Stm32f1VCPortClearFeature,
    Stm32f1VCPortSetEndPointFeature,
    Stm32f1VCPortSetDeviceFeature,
    Stm32f1VCPortSetDeviceAddress
  };

/**
 * @brief Device Descriptor structure
 */
ONE_DESCRIPTOR Device_Descriptor =
  {
    (uint8_t*)g_vectcVCPortDeviceDescriptor,
    VIRTUAL_COM_PORT_SIZ_DEVICE_DESC
  };

/**
 * @brief Configuration Descriptor structure
 */
ONE_DESCRIPTOR Config_Descriptor =
  {
    (uint8_t*)g_vectcVCPortConfigDescriptor,
    VIRTUAL_COM_PORT_SIZ_CONFIG_DESC
  };

/**
 * @brief String Descriptor structure
 */
ONE_DESCRIPTOR String_Descriptor[4] =
  {
    {(uint8_t*)g_vectcVCPortStringLangID, VIRTUAL_COM_PORT_SIZ_STRING_LANGID},
    {(uint8_t*)g_vectcVCPortStringVendor, VIRTUAL_COM_PORT_SIZ_STRING_VENDOR},
    {(uint8_t*)g_vectcVCPortStringProduct, VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT},
    {(uint8_t*)g_vectcVCPortStringSerial, VIRTUAL_COM_PORT_SIZ_STRING_SERIAL}
  };

/**
 * @}
 */


/** @defgroup STM32F1_VC_Properties_Private_FunctionPrototypes   STM32F1 VC Properties Private FunctionPrototypes
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_Properties_Private_Functions    STM32F1 VC Properties Private Functions
 * @{
 */

/**
 * @brief  Virtual Com Port initialization routine at usb level.
 * @retval None.
 */
void Stm32f1VCPortInit(void)
{

  /* Update the serial number string descriptor with the data from the unique
  ID*/
  Stm32f1VCGetSerialNum();

  pInformation->Current_Configuration = 0;

  /* Connect the device */
  PowerOn();

  /* Perform basic device initialization operations */
  USB_SIL_Init();

  bDeviceState = UNCONNECTED;
}

/**
  * @brief  Virtual Com Port reset routine
  * @retval None
  */

void Stm32f1VCPortReset(void)
{
  /* Set Virtual_Com_Port DEVICE as not configured */
  pInformation->Current_Configuration = 0;

  /* Current Feature initialization */
  pInformation->Current_Feature = g_vectcVCPortConfigDescriptor[7];

  /* Set Virtual_Com_Port DEVICE with the default Interface*/
  pInformation->Current_Interface = 0;

#ifdef STM32F10X_CL
  /* EP0 is already configured by USB_SIL_Init() function */

  /* Init EP1 IN as Bulk endpoint */
  OTG_DEV_EP_Init(EP1_IN, OTG_DEV_EP_TYPE_BULK, VIRTUAL_COM_PORT_DATA_SIZE);

  /* Init EP2 IN as Interrupt endpoint */
  OTG_DEV_EP_Init(EP2_IN, OTG_DEV_EP_TYPE_INT, VIRTUAL_COM_PORT_INT_SIZE);

  /* Init EP3 OUT as Bulk endpoint */
  OTG_DEV_EP_Init(EP3_OUT, OTG_DEV_EP_TYPE_BULK, VIRTUAL_COM_PORT_DATA_SIZE);
#else

  SetBTABLE(BTABLE_ADDRESS);

  /* Initialize Endpoint 0 */
  SetEPType(ENDP0, EP_CONTROL);
  SetEPTxStatus(ENDP0, EP_TX_STALL);
  SetEPRxAddr(ENDP0, ENDP0_RXADDR);
  SetEPTxAddr(ENDP0, ENDP0_TXADDR);
  Clear_Status_Out(ENDP0);
  SetEPRxCount(ENDP0, Device_Property.MaxPacketSize);
  SetEPRxValid(ENDP0);

  /* Initialize Endpoint 1 */
  SetEPType(ENDP1, EP_BULK);
  SetEPTxAddr(ENDP1, ENDP1_TXADDR);
  SetEPTxStatus(ENDP1, EP_TX_NAK);
  SetEPRxStatus(ENDP1, EP_RX_DIS);

  /* Initialize Endpoint 2 */
  SetEPType(ENDP2, EP_INTERRUPT);
  SetEPTxAddr(ENDP2, ENDP2_TXADDR);
  SetEPRxStatus(ENDP2, EP_RX_DIS);
  SetEPTxStatus(ENDP2, EP_TX_NAK);

  /* Initialize Endpoint 3 */
  SetEPType(ENDP3, EP_BULK);
  SetEPRxAddr(ENDP3, ENDP3_RXADDR);
  SetEPRxCount(ENDP3, VIRTUAL_COM_PORT_DATA_SIZE);
  SetEPRxStatus(ENDP3, EP_RX_VALID);
  SetEPTxStatus(ENDP3, EP_TX_DIS);

  /* Set this device to response on default address */
  SetDeviceAddress(0);
#endif /* STM32F10X_CL */

  bDeviceState = ATTACHED;
}


/**
  * @brief  Udpade the device state to configured
  * @retval None
  */

void Stm32f1VCPortSetConfiguration(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
}


/**
  * @brief  Udpade the device state to addressed
  * @retval None
  */

void Stm32f1VCPortSetDeviceAddress (void)
{
  bDeviceState = ADDRESSED;
}


/**
  * @brief  Virtual COM Port Status In Routine
  * @retval None
  */

void Stm32f1VCPortStatusIn(void)
{
  if (Request == SET_LINE_CODING)
  {
    Request = 0;
  }
}

/**
  * @brief  Virtual COM Port Status OUT Routine
  * @retval None
  */

void Stm32f1VCPortStatusOut(void)
{}


/**
  * @brief  Handle the data class specific requests
  * @param  RequestNo Request Nb
  * @retval RESULT:  Specifies the result of the operation
  *    This parameter can be one of the following parameters:
  *   @arg USB_UNSUPPORT
  *   @arg USB_SUCCESS
  */

RESULT Stm32f1VCPortDataSetup(uint8_t RequestNo)
{
  uint8_t    *(*CopyRoutine)(uint16_t);

  CopyRoutine = NULL;

  if (RequestNo == GET_LINE_CODING)
  {
    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))
    {
      CopyRoutine = Stm32f1VCPortGetLineCoding;
    }
  }
  else if (RequestNo == SET_LINE_CODING)
  {
    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))
    {
      CopyRoutine = Stm32f1VCPortSetLineCoding;
    }
    Request = SET_LINE_CODING;
  }

  if (CopyRoutine == NULL)
  {
    return USB_UNSUPPORT;
  }

  pInformation->Ctrl_Info.CopyData = CopyRoutine;
  pInformation->Ctrl_Info.Usb_wOffset = 0;
  (*CopyRoutine)(0);
  return USB_SUCCESS;
}


/**
  * @brief  Handle the no data class specific requests
  * @param  RequestNo Nb
  * @retval RESULT:  Specifies the result of the operation
  *    This parameter can be one of the following parameters:
  *   @arg USB_UNSUPPORT
  *   @arg USB_SUCCESS
  */

RESULT Stm32f1VCPortNoDataSetup(uint8_t RequestNo)
{

  if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))
  {
    if (RequestNo == SET_COMM_FEATURE)
    {
      return USB_SUCCESS;
    }
    else if (RequestNo == SET_CONTROL_LINE_STATE)
    {
      return USB_SUCCESS;
    }
  }

  return USB_UNSUPPORT;
}


/**
  * @brief  Gets the device descriptor
  * @param  Length
  * @retval The address of the device descriptor
  */

uint8_t *Stm32f1VCPortGetDeviceDescriptor(uint16_t Length)
{
  return Standard_GetDescriptorData(Length, &Device_Descriptor);
}


/**
  * @brief  Get the configuration descriptor
  * @param  Length
  * @retval The address of the configuration descriptor
  */

uint8_t *Stm32f1VCPortGetConfigDescriptor(uint16_t Length)
{
  return Standard_GetDescriptorData(Length, &Config_Descriptor);
}


/**
  * @brief  Gets the string descriptors according to the needed index
  * @param  Length
  * @retval The address of the string descriptors
  */

uint8_t *Stm32f1VCPortGetStringDescriptor(uint16_t Length)
{
  uint8_t wValue0 = pInformation->USBwValue0;
  if (wValue0 > 4)
  {
    return NULL;
  }
  else
  {
    return Standard_GetDescriptorData(Length, &String_Descriptor[wValue0]);
  }
}



/**
  * @brief  Gets the interface and the alternate setting according to the supported one
  * @param  Interface : interface number.
  * @param  AlternateSetting : Alternate Setting number.
  * @retval RESULT:  Specifies the result of the operation
  *    This parameter can be one of the following parameters:
  *   @arg USB_UNSUPPORT
  *   @arg USB_SUCCESS
  */

RESULT Stm32f1VCPortGetInterfaceSetting(uint8_t Interface, uint8_t AlternateSetting)
{
  if (AlternateSetting > 0)
  {
    return USB_UNSUPPORT;
  }
  else if (Interface > 1)
  {
    return USB_UNSUPPORT;
  }
  return USB_SUCCESS;
}


/**
  * @brief  Send the linecoding structure to the PC host
  * @param  Length
  * @retval Linecoding structure base address
  */

uint8_t *Stm32f1VCPortGetLineCoding(uint16_t Length)
{
  if (Length == 0)
  {
    pInformation->Ctrl_Info.Usb_wLength = sizeof(linecoding);
    return NULL;
  }
  return(uint8_t *)&linecoding;
}

/**
  * @brief  Set the linecoding structure fields
  * @param  Length
  * @retval Linecoding structure base address
  */

uint8_t *Stm32f1VCPortSetLineCoding(uint16_t Length)
{
  if (Length == 0)
  {
    pInformation->Ctrl_Info.Usb_wLength = sizeof(linecoding);
    return NULL;
  }
  return(uint8_t *)&linecoding;
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


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

