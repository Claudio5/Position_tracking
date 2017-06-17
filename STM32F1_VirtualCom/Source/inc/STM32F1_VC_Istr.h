/**
 * @file    STM32F1_VC_Istr.h
 * @author  MSH RF/ART Team IMS-Systems Lab
 * @version V1.0.0
 * @date    August 4, 2011
 * @brief   Interrupt Status Register header.
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
#ifndef __STM32F1_VC_ISTR_H
#define __STM32F1_VC_ISTR_H



#ifdef __cplusplus
extern "C" {
#endif



/** @defgroup STM32F1_Virtual_Com    STM32F1 Virtual Com
 * @{
 */


/** @defgroup STM32F1_VC_InterruptStatus   STM32F1 VC InterruptStatus
 * @brief Interrupt status register module for Virtual COM Port Device.
 * @details See the file <i>@ref STM32F1_VC_Istr.h</i> for more details.
 * @{
 */


/** @defgroup STM32F1_VC_InterruptStatus_Exported_Types    STM32F1 VC InterruptStatus Exported_Types
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_InterruptStatus_Exported_Constants    STM32F1 VC InterruptStatus Exported Constants
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_InterruptStatus_Exported_Macros   STM32F1 VC InterruptStatus Exported Macros
 * @{
 */

/**
 * @}
 */


/** @defgroup STM32F1_VC_InterruptStatus_Exported_Functions    STM32F1 VC InterruptStatus Exported Functions
 * @{
 */
#ifndef STM32F10X_CL
 void Stm32f1VCIntServRoutine(void);
#else /* STM32F10X_CL */
 u32 STM32_PCD_OTG_ISR_Handler(void);
#endif /* STM32F10X_CL */

/* function prototypes Automatically built defining related macros */

/**
 * @brief Function prototype for non-control endpoint input 1 service routines.
 */ 
void EP1_IN_Callback(void);

/**
 * @brief Function prototype for non-control endpoint input 2 service routines.
 */ 
void EP2_IN_Callback(void);

/**
 * @brief Function prototype for non-control endpoint input 3 service routines.
 */ 
void EP3_IN_Callback(void);

/**
 * @brief Function prototype for non-control endpoint input 4 service routines.
 */ 
void EP4_IN_Callback(void);

/**
 * @brief Function prototype for non-control endpoint input 5 service routines.
 */ 
void EP5_IN_Callback(void);

/**
 * @brief Function prototype for non-control endpoint input 6 service routines.
 */ 
void EP6_IN_Callback(void);

/**
 * @brief Function prototype for non-control endpoint input 7 service routines.
 */ 
void EP7_IN_Callback(void);

/**
 * @brief Function prototype for non-control endpoint output 1 service routines.
 */ 
void EP1_OUT_Callback(void);

/**
 * @brief Function prototype for non-control endpoint output 2 service routines.
 */ 
void EP2_OUT_Callback(void);

/**
 * @brief Function prototype for non-control endpoint output 3 service routines.
 */ 
void EP3_OUT_Callback(void);

/**
 * @brief Function prototype for non-control endpoint output 4 service routines.
 */ 
void EP4_OUT_Callback(void);

/**
 * @brief Function prototype for non-control endpoint output 5 service routines.
 */ 
void EP5_OUT_Callback(void);

/**
 * @brief Function prototype for non-control endpoint output 6 service routines.
 */ 
void EP6_OUT_Callback(void);

/**
 * @brief Function prototype for non-control endpoint output 7 service routines.
 */ 
void EP7_OUT_Callback(void);

#ifndef STM32F10X_CL

#ifdef CTR_CALLBACK
void CTR_Callback(void);
#endif

#ifdef DOVR_CALLBACK
void DOVR_Callback(void);
#endif

#ifdef ERR_CALLBACK
void ERR_Callback(void);
#endif

#ifdef WKUP_CALLBACK
void WKUP_Callback(void);
#endif

#ifdef SUSP_CALLBACK
void SUSP_Callback(void);
#endif

#ifdef RESET_CALLBACK
void RESET_Callback(void);
#endif

#ifdef SOF_CALLBACK
void SOF_Callback(void);
#endif

#ifdef ESOF_CALLBACK
void ESOF_Callback(void);
#endif

#else /* STM32F10X_CL */

/* Interrupt subroutines user callbacks prototypes.
   These callbacks are called into the respective interrupt sunroutine functinos
   and can be tailored for various user application purposes.
     Note: Make sure that the correspondant interrupt is enabled through the
     definition in usb_conf.h file */
void INTR_MODEMISMATCH_Callback(void);
void INTR_SOFINTR_Callback(void);
void INTR_RXSTSQLVL_Callback(void);
void INTR_NPTXFEMPTY_Callback(void);
void INTR_GINNAKEFF_Callback(void);
void INTR_GOUTNAKEFF_Callback(void);
void INTR_ERLYSUSPEND_Callback(void);
void INTR_USBSUSPEND_Callback(void);
void INTR_USBRESET_Callback(void);
void INTR_ENUMDONE_Callback(void);
void INTR_ISOOUTDROP_Callback(void);
void INTR_EOPFRAME_Callback(void);
void INTR_EPMISMATCH_Callback(void);
void INTR_INEPINTR_Callback(void);
void INTR_OUTEPINTR_Callback(void);
void INTR_INCOMPLISOIN_Callback(void);
void INTR_INCOMPLISOOUT_Callback(void);
void INTR_WKUPINTR_Callback(void);

/* Isochronous data update */
void INTR_RXSTSQLVL_ISODU_Callback(void);

#endif /* STM32F10X_CL */

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
