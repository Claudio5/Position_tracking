/**
 * @file    STM32F1_VC_Prop.h
 * @author  MSH RF/ART Team IMS-Systems Lab
 * @version V1.0.0
 * @date    August 4, 2011
 * @brief   Header file for all processing related to Virtual COM Port.
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
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F1_VC_PROP_H
#define __STM32F1_VC_PROP_H


#ifdef __cplusplus
extern "C" {
#endif



/** @defgroup STM32F1_Virtual_Com    STM32F1 Virtual Com
 * @{
 */


/** @defgroup STM32F1_VC_Properties    STM32F1 VC Properties
 * @brief Properties and configuration module for Virtual COM Port Device.
 * @details See the file <i>@ref STM32F1_VC_Prop.h</i> for more details.
 * @{
 */


/** @defgroup STM32F1_VC_Properties_Exported_Types   STM32F1 VC Properties Exported Types
 * @{
 */

/**
 * @brief USB line coding structure
 */
typedef struct
{
  uint32_t bitrate;     /*!< Bit Rate */
  uint8_t format;       /*!< Format */
  uint8_t paritytype;   /*!< Parity */
  uint8_t datatype;     /*!< Data Type */
}LineCoding;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/

/**
 * @brief Stm32f1VCPortGetConfiguration 
 */
#define Stm32f1VCPortGetConfiguration          NOP_Process

/**
 * @brief Stm32f1VCPortSetConfiguration 
 */
//#define Stm32f1VCPortSetConfiguration          NOP_Process

/**
 * @brief SdkEvalVCPortGetInterface 
 */
#define SdkEvalVCPortGetInterface              NOP_Process

/**
 * @brief SdkEvalVCPortSetInterface 
 */
#define SdkEvalVCPortSetInterface              NOP_Process

/**
 * @brief Stm32f1VCPortGetStatus 
 */
#define Stm32f1VCPortGetStatus                 NOP_Process

/**
 * @brief Stm32f1VCPortClearFeature 
 */
#define Stm32f1VCPortClearFeature              NOP_Process

/**
 * @brief Stm32f1VCPortSetEndPointFeature 
 */
#define Stm32f1VCPortSetEndPointFeature        NOP_Process

/**
 * @brief Stm32f1VCPortSetDeviceFeature 
 */
#define Stm32f1VCPortSetDeviceFeature          NOP_Process

/**
 * @brief Stm32f1VCPortSetDeviceAddress 
 */
//#define Stm32f1VCPortSetDeviceAddress          NOP_Process


/**
 * @brief SEND_ENCAPSULATED_COMMAND Value 
 */
#define SEND_ENCAPSULATED_COMMAND   0x00

/**
 * @brief GET_ENCAPSULATED_RESPONSE Value 
 */
#define GET_ENCAPSULATED_RESPONSE   0x01

/**
 * @brief SET_COMM_FEATURE Value 
 */
#define SET_COMM_FEATURE            0x02

/**
 * @brief GET_COMM_FEATURE Value 
 */
#define GET_COMM_FEATURE            0x03

/**
 * @brief CLEAR_COMM_FEATURE Value 
 */
#define CLEAR_COMM_FEATURE          0x04

/**
 * @brief SET_LINE_CODING Value 
 */
#define SET_LINE_CODING             0x20

/**
 * @brief GET_LINE_CODING Value 
 */
#define GET_LINE_CODING             0x21

/**
 * @brief SET_CONTROL_LINE_STATE Value 
 */
#define SET_CONTROL_LINE_STATE      0x22

/**
 * @brief SEND_BREAK Value 
 */
#define SEND_BREAK                  0x23

/**
 * @}
 */


/** @defgroup STM32F1_VC_Properties_Exported_Constants   STM32F1 VC Properties Exported Constants
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_Properties_Exported_Macros    STM32F1 VC Properties Exported Macros
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_Properties_Exported_Functions   STM32F1 VC Properties Exported Functions
 * @{
 */
void Stm32f1VCPortInit(void);
void Stm32f1VCPortReset(void);
void Stm32f1VCPortSetConfiguration(void);
void Stm32f1VCPortSetDeviceAddress (void);
void Stm32f1VCPortStatusIn (void);
void Stm32f1VCPortStatusOut (void);
RESULT Stm32f1VCPortDataSetup(uint8_t);
RESULT Stm32f1VCPortNoDataSetup(uint8_t);
RESULT Stm32f1VCPortGetInterfaceSetting(uint8_t Interface, uint8_t AlternateSetting);
uint8_t *Stm32f1VCPortGetDeviceDescriptor(uint16_t );
uint8_t *Stm32f1VCPortGetConfigDescriptor(uint16_t);
uint8_t *Stm32f1VCPortGetStringDescriptor(uint16_t);

uint8_t *Stm32f1VCPortGetLineCoding(uint16_t Length);
uint8_t *Stm32f1VCPortSetLineCoding(uint16_t Length);


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

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/

