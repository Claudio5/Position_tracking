/**
  * @file    STM32F1_VC_Desc.c
  * @author  MCD Application Team & SystemsLab
  * @version V3.2.1
  * @date    09/20/2010
  * @brief   Descriptors for Virtual Com Port.
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
#include "usb_lib.h"
#include "STM32F1_VC_Desc.h"



/** @addtogroup STM32F1_Virtual_Com
 * @{
 */


/** @addtogroup STM32F1_VC_Descriptor
 * @{
 */


/** @defgroup STM32F1_VC_Descriptor_Private_TypesDefinitions   STM32F1 VC Descriptor Private TypesDefinitions
 * @{
 */

/**
 * @}
*/


/** @defgroup STM32F1_VC_Descriptor_Private_Defines      STM32F1 VC Descriptor Private Defines
* @{
*/

/**
* @}
*/


/** @defgroup STM32F1_VC_Descriptor_Private_Macros      STM32F1 VC Descriptor Private Macros
* @{
*/

/**
* @}
*/


/** @defgroup STM32F1_VC_Descriptor_Private_Variables    STM32F1 VC Descriptor Private Variables
* @{
*/

/**
* @brief   Virtual Com Port Device Descriptor
*/
const uint8_t g_vectcVCPortDeviceDescriptor[] =
{
  0x12,   /* bLength */
  USB_DEVICE_DESCRIPTOR_TYPE,     /* bDescriptorType */
  0x00,
  0x02,   /* bcdUSB = 2.00 */
  0x02,   /* bDeviceClass: CDC */
  0x00,   /* bDeviceSubClass */
  0x00,   /* bDeviceProtocol */
  0x40,   /* bMaxPacketSize0 */
  0x83,
  0x04,   /* idVendor = 0x0483 */
  PRODUCT_ID_LSB,
  PRODUCT_ID_MSB,  /* idProduct (defined in the conf file) */
  0x00,
  0x02,   /* bcdDevice = 2.00 */
  1,              /* Index of string descriptor describing manufacturer */
  2,              /* Index of string descriptor describing product */
  3,              /* Index of string descriptor describing the device's serial number */
  0x01    /* bNumConfigurations */
};

/**
* @brief   Virtual Com Port Configuration Descriptor
*/
const uint8_t g_vectcVCPortConfigDescriptor[] =
{
  /*Configuation Descriptor*/
  0x09,   /* bLength: Configuation Descriptor size */
  USB_CONFIGURATION_DESCRIPTOR_TYPE,      /* bDescriptorType: Configuration */
  VIRTUAL_COM_PORT_SIZ_CONFIG_DESC,       /* wTotalLength:no of returned bytes */
  0x00,
  0x02,   /* bNumInterfaces: 2 interface */
  0x01,   /* bConfigurationValue: Configuration value */
  0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
  0xC0,   /* bmAttributes: self powered */
  0x32,   /* MaxPower 0 mA */
  /*Interface Descriptor*/
  0x09,   /* bLength: Interface Descriptor size */
  USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  0x00,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x01,   /* bNumEndpoints: One endpoints used */
  0x02,   /* bInterfaceClass: Communication Interface Class */
  0x02,   /* bInterfaceSubClass: Abstract Control Model */
  0x01,   /* bInterfaceProtocol: Common AT commands */
  0x00,   /* iInterface: */
  /*Header Functional Descriptor*/
  0x05,   /* bLength: Endpoint Descriptor size */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x00,   /* bDescriptorSubtype: Header Func Desc */
  0x10,   /* bcdCDC: spec release number */
  0x01,
  /*Call Managment Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x01,   /* bDescriptorSubtype: Call Management Func Desc */
  0x00,   /* bmCapabilities: D0+D1 */
  0x01,   /* bDataInterface: 1 */
  /*ACM Functional Descriptor*/
  0x04,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,   /* bmCapabilities */
  /*Union Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x06,   /* bDescriptorSubtype: Union func desc */
  0x00,   /* bMasterInterface: Communication class interface */
  0x01,   /* bSlaveInterface0: Data Class Interface */
  /*Endpoint 2 Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
  0x82,   /* bEndpointAddress: (IN2) */
  0x03,   /* bmAttributes: Interrupt */
  VIRTUAL_COM_PORT_INT_SIZE,      /* wMaxPacketSize: */
  0x00,
  0xFF,   /* bInterval: */
  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: */
  0x01,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0x0A,   /* bInterfaceClass: CDC */
  0x00,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  0x00,   /* iInterface: */
  /*Endpoint 3 Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
  0x03,   /* bEndpointAddress: (OUT3) */
  0x02,   /* bmAttributes: Bulk */
  VIRTUAL_COM_PORT_DATA_SIZE,             /* wMaxPacketSize: */
  0x00,
  0x00,   /* bInterval: ignore for Bulk transfer */
  /*Endpoint 1 Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
  0x81,   /* bEndpointAddress: (IN1) */
  0x02,   /* bmAttributes: Bulk */
  VIRTUAL_COM_PORT_DATA_SIZE,             /* wMaxPacketSize: */
  0x00,
  0x00    /* bInterval */
};

/**
* @brief USB Port String Descriptors
*/
const uint8_t g_vectcVCPortStringLangID[VIRTUAL_COM_PORT_SIZ_STRING_LANGID] =
{
  VIRTUAL_COM_PORT_SIZ_STRING_LANGID,
  USB_STRING_DESCRIPTOR_TYPE,
  0x09,
  0x04 /* LangID = 0x0409: U.S. English */
};

/**
* @brief USB Port String Vendor Descriptors
*/
const uint8_t g_vectcVCPortStringVendor[VIRTUAL_COM_PORT_SIZ_STRING_VENDOR] =
{
  VIRTUAL_COM_PORT_SIZ_STRING_VENDOR,     /* Size of Vendor string */
  USB_STRING_DESCRIPTOR_TYPE,             /* bDescriptorType*/
  /* Manufacturer: "STMicroelectronics" */
  'S', 0, 'T', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'e', 0,
  'l', 0, 'e', 0, 'c', 0, 't', 0, 'r', 0, 'o', 0, 'n', 0, 'i', 0,
  'c', 0, 's', 0
};

/**
* @brief USB Port String Product Descriptors
*/
const uint8_t g_vectcVCPortStringProduct[VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT] =
{
  VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT,          /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
  /* Product name (defined in the conf file) */
  STRING_PRODUCT_NAME
    
};

/**
* @brief USB Port String Serial Descriptors
*/
uint8_t g_vectcVCPortStringSerial[VIRTUAL_COM_PORT_SIZ_STRING_SERIAL] =
{
  VIRTUAL_COM_PORT_SIZ_STRING_SERIAL,           /* bLength */
  USB_STRING_DESCRIPTOR_TYPE,                   /* bDescriptorType */
  'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0, '1', 0, '0', 0
};

/**
 * @}
 */


/** @defgroup STM32F1_VC_Descriptor_Private_FunctionPrototypes   STM32F1 VC Descriptor Private FunctionPrototypes
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_Descriptor_Private_Functions    STM32F1 VC Descriptor Private Functions
 * @{
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

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
