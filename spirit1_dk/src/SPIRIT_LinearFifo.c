/**
* @file    SPIRIT_LinearFifo.c
* @author  VMA division - AMS
* @version 3.2.0
* @date    February 1, 2015
* @brief   Configuration and management of SPIRIT Fifo.
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


/* Includes ------------------------------------------------------------------*/
#include "SPIRIT_LinearFifo.h"
#include "MCU_Interface.h"


/**
 * @addtogroup SPIRIT_Libraries
 * @{
 */


/**
 * @addtogroup SPIRIT_LinearFifo
 * @{
 */


/**
 * @defgroup LinearFifo_Private_TypesDefinitions        Linear FIFO Private Types Definitions
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup LinearFifo_Private_Defines                 Linear FIFO Private Defines
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup LinearFifo_Private_Macros                  Linear FIFO Private Macros
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup LinearFifo_Private_Variables               Linear FIFO Private Variables
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup LinearFifo_Private_FunctionPrototypes      Linear FIFO Private Function Prototypes
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup LinearFifo_Private_Functions               Linear FIFO Private Functions
 * @{
 */

/**
 * @brief  Returns the number of elements in the Rx FIFO.
 * @param  None.
 * @retval uint8_t Number of elements in the Rx FIFO.
 */
uint8_t SpiritLinearFifoReadNumElementsRxFifo(void)
{
  uint8_t tempRegValue;

  /* Reads the register value */
  g_xStatus = SpiritSpiReadRegisters(LINEAR_FIFO_STATUS0_BASE, 1, &tempRegValue);

  /* Build and return value */
  return (tempRegValue & 0x7F);

}


/**
 * @brief  Returns the number of elements in the Tx FIFO.
 * @param  None.
 * @retval uint8_t Number of elements in the Tx FIFO.
 */
uint8_t SpiritLinearFifoReadNumElementsTxFifo(void)
{
  uint8_t tempRegValue;

  /* Reads the number of elements in TX FIFO and return the value */
  g_xStatus = SpiritSpiReadRegisters(LINEAR_FIFO_STATUS1_BASE, 1, &tempRegValue);

  /* Build and return value */
  return (tempRegValue & 0x7F);

}


/**
 * @brief  Sets the almost full threshold for the Rx FIFO. When the number of elements in RX FIFO reaches this value an interrupt can be generated to the MCU.
 * @note   The almost full threshold is encountered from the top of the FIFO. For example, if it is set to 7 the almost
 *         full FIFO irq will be raised when the number of elements is equals to 96-7 = 89.
 * @param  cThrRxFifo almost full threshold.
 * 	   This parameter is an uint8_t.
 * @retval None.
 */
void SpiritLinearFifoSetAlmostFullThresholdRx(uint8_t cThrRxFifo)
{
  uint8_t tempRegValue;

  /* Check the parameters */
  s_assert_param(IS_FIFO_THR(cThrRxFifo));

  /* Build the register value */
  tempRegValue = cThrRxFifo & 0x7F;

  /* Writes the Almost Full threshold for RX in the corresponding register */
  g_xStatus = SpiritSpiWriteRegisters(FIFO_CONFIG3_RXAFTHR_BASE, 1, &tempRegValue);

}


/**
 * @brief  Returns the almost full threshold for RX FIFO.
 * @note   The almost full threshold is encountered from the top of the FIFO. For example, if it is 7 the almost
 *         full FIFO irq will be raised when the number of elements is equals to 96-7 = 89.
 * @param  None.
 * @retval uint8_t Almost full threshold for Rx FIFO.
 */
uint8_t SpiritLinearFifoGetAlmostFullThresholdRx(void)
{
  uint8_t tempRegValue;

  /* Reads the almost full threshold for RX FIFO and return the value */
  g_xStatus = SpiritSpiReadRegisters(FIFO_CONFIG3_RXAFTHR_BASE, 1, &tempRegValue);

  /* Build and return value */
  return (tempRegValue & 0x7F);

}


/**
 * @brief  Sets the almost empty threshold for the Rx FIFO. When the number of elements in RX FIFO reaches this value an interrupt can be generated to the MCU.
 * @param  cThrRxFifo almost empty threshold.
 * 	   This parameter is an uint8_t.
 * @retval None.
 */
void SpiritLinearFifoSetAlmostEmptyThresholdRx(uint8_t cThrRxFifo)
{
  uint8_t tempRegValue;

  /* Check the parameters */
  s_assert_param(IS_FIFO_THR(cThrRxFifo));

  /* Build the register value */
  tempRegValue = cThrRxFifo & 0x7F;

  /* Writes the Almost Empty threshold for RX in the corresponding register */
  g_xStatus = SpiritSpiWriteRegisters(FIFO_CONFIG2_RXAETHR_BASE, 1, &tempRegValue);

}


/**
 * @brief  Returns the almost empty threshold for Rx FIFO.
 * @param  None.
 * @retval uint8_t Almost empty threshold for Rx FIFO.
 */
uint8_t SpiritLinearFifoGetAlmostEmptyThresholdRx(void)
{
  uint8_t tempRegValue;

  /* Reads the almost empty threshold for RX FIFO and returns the value */
  g_xStatus = SpiritSpiReadRegisters(FIFO_CONFIG2_RXAETHR_BASE, 1, &tempRegValue);

  /* Build and return value */
  return (tempRegValue & 0x7F);

}


/**
 * @brief  Sets the almost full threshold for the Tx FIFO. When the number of elements in TX FIFO reaches this value an interrupt can be generated to the MCU.
 * @note   The almost full threshold is encountered from the top of the FIFO. For example, if it is set to 7 the almost
 *         full FIFO irq will be raised when the number of elements is equals to 96-7 = 89.
 * @param  cThrTxFifo almost full threshold.
 * 	   This parameter is an uint8_t.
 * @retval None.
 */
void SpiritLinearFifoSetAlmostFullThresholdTx(uint8_t cThrTxFifo)
{
  uint8_t tempRegValue;

  /* Check the parameters */
  s_assert_param(IS_FIFO_THR(cThrTxFifo));

  /* Reads the register value */
  g_xStatus = SpiritSpiReadRegisters(FIFO_CONFIG1_TXAFTHR_BASE, 1, &tempRegValue);

  /* Build the register value */
  tempRegValue &= 0x80;
  tempRegValue |= cThrTxFifo;

  /* Writes the Almost Full threshold for Tx in the corresponding register */
  g_xStatus = SpiritSpiWriteRegisters(FIFO_CONFIG1_TXAFTHR_BASE, 1, &tempRegValue);

}


/**
 * @brief  Returns the almost full threshold for Tx FIFO.
 * @note   The almost full threshold is encountered from the top of the FIFO. For example, if it is set to 7 the almost
 *         full FIFO irq will be raised when the number of elements is equals to 96-7 = 89.
 * @param  None.
 * @retval uint8_t Almost full threshold for Tx FIFO.
 */
uint8_t SpiritLinearFifoGetAlmostFullThresholdTx(void)
{
  uint8_t tempRegValue;

  /* Reads the almost full threshold for Tx FIFO and returns the value */
  g_xStatus = SpiritSpiReadRegisters(FIFO_CONFIG1_TXAFTHR_BASE, 1, &tempRegValue);

  /* Build and returns value */
  return (tempRegValue & 0x7F);

}


/**
 * @brief  Sets the almost empty threshold for the Tx FIFO. When the number of elements in Tx FIFO reaches this value an interrupt can can be generated to the MCU.
 * @param  cThrTxFifo: almost empty threshold.
 *         This parameter is an uint8_t.
 * @retval None.
 */
void SpiritLinearFifoSetAlmostEmptyThresholdTx(uint8_t cThrTxFifo)
{
  uint8_t tempRegValue;

  /* Check the parameters */
  s_assert_param(IS_FIFO_THR(cThrTxFifo));

  /* Reads the register value */
  g_xStatus = SpiritSpiReadRegisters(FIFO_CONFIG0_TXAETHR_BASE, 1, &tempRegValue);

  /* Build the register value */
  tempRegValue &= 0x80;
  tempRegValue |= cThrTxFifo;

  /* Writes the Almost Empty threshold for Tx in the corresponding register */
  g_xStatus = SpiritSpiWriteRegisters(FIFO_CONFIG0_TXAETHR_BASE, 1, &tempRegValue);

}


/**
 * @brief  Returns the almost empty threshold for Tx FIFO.
 * @param  None.
 * @retval uint8_t Almost empty threshold for Tx FIFO.
 */
uint8_t SpiritLinearFifoGetAlmostEmptyThresholdTx(void)
{
  uint8_t tempRegValue;

  /* Reads the almost empty threshold for TX FIFO and returns the value */
  g_xStatus = SpiritSpiReadRegisters(FIFO_CONFIG0_TXAETHR_BASE, 1, &tempRegValue);

  /* Build and return value */
  return (tempRegValue & 0x7F);

}


/**
 *@}
 */

/**
 *@}
 */


/**
 *@}
 */



/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
