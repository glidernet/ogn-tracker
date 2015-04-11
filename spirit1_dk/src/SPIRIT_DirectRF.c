/**
* @file    SPIRIT_DirectRF.c
* @author  VMA division - AMS
* @version 3.2.0
* @date    February 1, 2015
* @brief   Configuration and management of SPIRIT direct transmission / receive modes.
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
#include "SPIRIT_DirectRF.h"
#include "MCU_Interface.h"



/**
 * @addtogroup SPIRIT_Libraries
 * @{
 */


/**
 * @addtogroup SPIRIT_DirectRf
 * @{
 */


/**
 * @defgroup DirectRf_Private_TypesDefinitions          Direct RF Private Types Definitions
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup DirectRf_Private_Defines                   Direct RF Private Defines
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup DirectRf_Private_Macros                    Direct RF Private Macros
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup DirectRf_Private_Variables                 Direct RF Private Variables
 * @{
 */

/**
 *@}
 */



/**
 * @defgroup DirectRf_Private_FunctionPrototypes        Direct RF Private Function Prototypes
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup DirectRf_Private_Functions                 Direct RF Private Functions
 * @{
 */

/**
 * @brief  Sets the DirectRF RX mode of SPIRIT.
 * @param  xDirectRx code of the desired mode.
 *         This parameter can be any value of @ref DirectRx.
 * @retval None.
 */
void SpiritDirectRfSetRxMode(DirectRx xDirectRx)
{
  uint8_t tempRegValue;

  /* Check the parameters */
  s_assert_param(IS_DIRECT_RX(xDirectRx));

  /* Reads the register value */
  SpiritSpiReadRegisters(PCKTCTRL3_BASE, 1, &tempRegValue);

  /* Build the value to be stored */
  tempRegValue &= ~PCKTCTRL3_RX_MODE_MASK;
  tempRegValue |= (uint8_t)xDirectRx;

  /* Writes value on register */
  g_xStatus = SpiritSpiWriteRegisters(PCKTCTRL3_BASE, 1, &tempRegValue);

}


/**
 * @brief  Returns the DirectRF RX mode of SPIRIT.
 * @param  None.
 * @retval DirectRx Direct Rx mode.
 */
DirectRx SpiritDirectRfGetRxMode(void)
{
  uint8_t tempRegValue;

  /* Reads the register value and mask the RX_Mode field */
  g_xStatus = SpiritSpiReadRegisters(PCKTCTRL3_BASE, 1, &tempRegValue);

  /* Rebuild and return value */
  return (DirectRx)(tempRegValue & 0x30);

}


/**
 * @brief  Sets the TX mode of SPIRIT.
 * @param  xDirectTx code of the desired source.
 *         This parameter can be any value of @ref DirectTx.
 * @retval None.
 */
void SpiritDirectRfSetTxMode(DirectTx xDirectTx)
{
  uint8_t tempRegValue;

  /* Check the parameters */
  s_assert_param(IS_DIRECT_TX(xDirectTx));

  /* Reads the register value */
  SpiritSpiReadRegisters(PCKTCTRL1_BASE, 1, &tempRegValue);

  /* Build the value to be stored */
  tempRegValue &= ~PCKTCTRL1_TX_SOURCE_MASK;
  tempRegValue |= (uint8_t)xDirectTx;

  /* Writes value on register */
  g_xStatus = SpiritSpiWriteRegisters(PCKTCTRL1_BASE, 1, &tempRegValue);

}


/**
 * @brief  Returns the DirectRF TX mode of SPIRIT.
 * @param  None.
 * @retval DirectTx Direct Tx mode.
 */
DirectTx SpiritDirectRfGetTxMode(void)
{
  uint8_t tempRegValue;

  /* Reads the register value and mask the RX_Mode field */
  g_xStatus = SpiritSpiReadRegisters(PCKTCTRL1_BASE, 1, &tempRegValue);

  /* Returns value */
  return (DirectTx)(tempRegValue & 0x0C);

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
