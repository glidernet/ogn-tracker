/**
* @file    SPIRIT_DirectRF.h
* @author  VMA division - AMS
* @version 3.2.0
* @date    February 1, 2015
* @brief   Configuration and management of SPIRIT direct transmission / receive modes.
* @details
*
* This module contains functions to manage the direct Tx/Rx mode.
* The user can choose the way to send data to Spirit through the
* enumerative types <i>@ref DirectTx</i>/<i>@ref DirectRx</i>.
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
#ifndef __SPIRIT1_DIRECT_RF_H
#define __SPIRIT1_DIRECT_RF_H

/* Includes ------------------------------------------------------------------*/

#include "SPIRIT_Regs.h"
#include "SPIRIT_Types.h"



#ifdef __cplusplus
 extern "C" {
#endif


/**
 * @addtogroup SPIRIT_Libraries
 * @{
 */


/**
 * @defgroup SPIRIT_DirectRf    Direct RF
 * @brief Configuration and management of SPIRIT direct transmission / receive modes.
 * @details See the file <i>@ref SPIRIT_DirectRF.h</i> for more details.
 * @{
 */

/**
 * @defgroup DirectRf_Exported_Types    Direct RF Exported Types
 * @{
 */

/**
 * @brief  Direct transmission mode enumeration for SPIRIT.
 */
typedef enum
{
  NORMAL_TX_MODE = 0x00,          /*!< Normal mode, no direct transmission is used */
  DIRECT_TX_FIFO_MODE = 0x04,     /*!< Source is FIFO: payload bits are continuously read from the TX FIFO */
  DIRECT_TX_GPIO_MODE = 0x08,     /*!< Source is GPIO: payload bits are continuously read from one of the GPIO ports and transmitted without any processing */
  PN9_TX_MODE = 0x0C              /*!< A pseudorandom binary sequence is generated internally */
}DirectTx;

#define IS_DIRECT_TX(MODE)  (((MODE) == NORMAL_TX_MODE) || \
			     ((MODE) == DIRECT_TX_FIFO_MODE) || \
                             ((MODE) == DIRECT_TX_GPIO_MODE)  || \
                             ((MODE) == PN9_TX_MODE))

/**
 * @brief  Direct receive mode enumeration for SPIRIT.
 */
typedef enum
{
  NORMAL_RX_MODE = 0x00,          /*!< Normal mode, no direct reception is used */
  DIRECT_RX_FIFO_MODE = 0x10,     /*!< Destination is FIFO: payload bits are continuously written to the RX FIFO and not subjected to any processing*/
  DIRECT_RX_GPIO_MODE = 0x20      /*!< Destination is GPIO: payload bits are continuously written to one of the GPIO ports and not subjected to any processing*/
}DirectRx;

#define IS_DIRECT_RX(MODE)  (((MODE) == NORMAL_RX_MODE) || \
		             ((MODE) == DIRECT_RX_FIFO_MODE) || \
		             ((MODE) == DIRECT_RX_GPIO_MODE))


/**
 *@}
 */


/**
 * @defgroup DirectRf_Exported_Constants        Direct RF Exported Constants
 * @{
 */


/**
 *@}
 */


/**
 * @defgroup DirectRf_Exported_Macros           Direct RF Exported Macros
 * @{
 */


/**
 *@}
 */


/**
 * @defgroup DirectRf_Exported_Functions        Direct RF Exported Functions
 * @{
 */

void SpiritDirectRfSetRxMode(DirectRx xDirectRx);
DirectRx SpiritDirectRfGetRxMode(void);
void SpiritDirectRfSetTxMode(DirectTx xDirectTx);
DirectTx SpiritDirectRfGetTxMode(void);

/**
 *@}
 */

/**
 *@}
 */


/**
 *@}
 */


#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
