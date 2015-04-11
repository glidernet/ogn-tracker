/**
* @file    SPIRIT_Management.h
* @author  VMA division - AMS
* @version 3.2.0
* @date    February 1, 2015
* @brief   The management layer for SPIRIT1 library.
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
* <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef SPIRIT_MANAGEMENT_H_
#define SPIRIT_MANAGEMENT_H_

/* Includes ------------------------------------------------------------------*/
#include "SPIRIT_Config.h"

#ifdef __cplusplus
  extern "C" {
#endif


/**
 * @addtogroup SPIRIT_Libraries
 * @{
 */


/**
 * @defgroup SPIRIT_MANAGEMENT          Management
 * @brief Workarounds for Spirit1.
 * @details See the file <i>@ref SPIRIT_Management.h</i> for more details.
 * @{
 */


/**
 * @addgroup SPIRIT_MANAGEMENT_FUNCTIONS
 * @{
 */

   


uint8_t SpiritManagementWaVcoCalibration(void);
void SpiritManagementWaCmdStrobeTx(void);
void SpiritManagementWaCmdStrobeRx(void);
void SpiritManagementWaTRxFcMem(uint32_t nDesiredFreq);
void SpiritManagementWaExtraCurrent(void);

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


 /******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/

