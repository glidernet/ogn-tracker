/**
* @file    SPIRIT_Commands.c
* @author  VMA division - AMS
* @version 3.2.0
* @date    February 1, 2015
* @brief   Management of SPIRIT Commands.
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
#include "SPIRIT_Commands.h"
#include "MCU_Interface.h"




/**
 * @addtogroup SPIRIT_Libraries
 * @{
 */


/**
 * @addtogroup SPIRIT_Commands
 * @{
 */


/**
 * @defgroup Commands_Private_TypesDefinitions  Commands Private TypesDefinitions
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Commands_Private_Defines           Commands Private Defines
 * @{
 */

/**
 *@}
 */

/**
 * @defgroup Commands_Private_Macros            Commands Private Macros
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Commands_Private_Variables         Commands Private Variables
 * @{
 */

/**
 *@}
 */



/**
 * @defgroup Commands_Private_FunctionPrototypes        Commands Private Function Prototypes
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Commands_Private_Functions                 Commands Private Functions
 * @{
 */

/**
 * @brief  Sends a specific command to SPIRIT.
 * @param  xCommandCode code of the command to send.
           This parameter can be any value of @ref SpiritCmd.
 * @retval None.
 */
void SpiritCmdStrobeCommand(SpiritCmd xCommandCode)
{
  /* Check the parameters */
  s_assert_param(IS_SPIRIT_CMD(xCommandCode));

  g_xStatus = SpiritSpiCommandStrobes((uint8_t) xCommandCode);
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
