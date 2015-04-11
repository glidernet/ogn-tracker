/**
* @file    SPIRIT_LinearFifo.h
* @author  VMA division - AMS
* @version 3.2.0
* @date    February 1, 2015
* @brief   Configuration and management of SPIRIT Fifo.
* @details
*
* This module allows the user to manage the linear FIFO. The functions exported
* here can be used to set the thresholds for the FIFO almost full / empty alarm
* interrupts or to get the total number of elements inside the FIFO.
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPIRIT_LINEAR_FIFO_H
#define __SPIRIT_LINEAR_FIFO_H


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
 * @defgroup SPIRIT_LinearFifo          Linear FIFO
 * @brief Configuration and management of SPIRIT FIFO.
 * @details See the file <i>@ref SPIRIT_LinearFifo.h</i> for more details.
 * @{
 */

/**
 * @defgroup LinearFifo_Exported_Types  Linear FIFO Exported Types
 * @{
 */


/**
 * @}
 */


/**
 * @defgroup LinearFifo_Exported_Constants      Linear FIFO Exported Constants
 * @{
 */
#define IS_FIFO_THR(VAL)  (VAL<=96)

/**
 * @}
 */


/**
 * @defgroup LinearFifo_Exported_Macros         Linear FIFO Exported Macros
 * @{
 */


/**
 * @}
 */


/**
 * @defgroup LinearFifo_Exported_Functions                      Linear FIFO Exported Functions
 * @{
 */

uint8_t SpiritLinearFifoReadNumElementsRxFifo(void);
uint8_t SpiritLinearFifoReadNumElementsTxFifo(void);
void SpiritLinearFifoSetAlmostFullThresholdRx(uint8_t cThrRxFifo);
uint8_t SpiritLinearFifoGetAlmostFullThresholdRx(void);
void SpiritLinearFifoSetAlmostEmptyThresholdRx(uint8_t cThrRxFifo);
uint8_t SpiritLinearFifoGetAlmostEmptyThresholdRx(void);
void SpiritLinearFifoSetAlmostFullThresholdTx(uint8_t cThrTxFifo);
uint8_t SpiritLinearFifoGetAlmostFullThresholdTx(void);
void SpiritLinearFifoSetAlmostEmptyThresholdTx(uint8_t cThrTxFifo);
uint8_t SpiritLinearFifoGetAlmostEmptyThresholdTx(void);

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
