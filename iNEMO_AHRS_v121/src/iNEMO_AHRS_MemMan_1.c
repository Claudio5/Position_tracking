/**
*
* \file    iNEMO_AHRS_MemMan_1.h
* \author  ART Team IMS-Systems Lab
* \version V1.2.1 [FW v2.0.0]
* \date    13/09/2010
* \brief   This file implements the Memory Manager of the AHRS of iNEMO
*
********************************************************************************
*
* \details
*  This file should be added to your Project any time you would use the 
*      standard malloc and free functions.
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
********************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __iNEMO_AHRS_MEMMAN_H
#define __iNEMO_AHRS_MEMMAN_H


/* Memory Allocation Functions ---------------------------------------------- */

#include <stdlib.h>

/**
********************************************************************************
* @brief Propagate the Error Covariance Matrix
* @param  pPoldMat  : old P Matrix  
* @param  pStateMat : the State Matrix 
* @param  pQMat     : the Q Covariance Matrix of noise on state 
* @param  pPnewMat  : the new P matrix 
* @retval Pointer to iNEMO_fMATRIX_TYPE
* @par Functions called:
* @ref iNEMO_fMatCreate
* @ref iNEMO_fMatMulMat
* @ref iNEMO_fMatMulMatMT
* @ref iNEMO_fMatAdd
* @ref iNEMO_fMatFree
*/
void *iNEMO_Malloc(size_t size)
{ 
  return malloc(size);
}

void iNEMO_Free(void *p)
{
  free(p);
}

#endif 
/* __iNEMO_AHRS_MEMMAN_H */