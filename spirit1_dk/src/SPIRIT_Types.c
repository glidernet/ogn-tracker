/**
* @file     SPIRIT_Types.c
* @author  VMA division - AMS
* @version 3.2.0
* @date    February 1, 2015
* @brief    This file provides functions to manage SPIRIT debug.
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
*
*/

/* Includes ------------------------------------------------------------------*/
#include "SPIRIT_Types.h"
#include "MCU_Interface.h"


/** @addtogroup SPIRIT_Libraries
 * @{
 */


/** @addtogroup SPIRIT_Types
 * @{
 */


/** @defgroup Types_Private_TypesDefinitions    Types Private Types Definitions
 * @{
 */


/**
 * @}
 */



/** @defgroup Types_Private_Defines             Types Private Defines
 * @{
 */


/**
 * @}
 */



/** @defgroup Types_Private_Macros               Types Private Macros
 * @{
 */


/**
 * @}
 */



/** @defgroup Types_Private_Variables             Types Private Variables
 * @{
 */

/**
 * @brief  Spirit Status global variable.
 *         This global variable of @ref SpiritStatus type is updated on every SPI transaction
 *         to maintain memory of Spirit Status.
 */

volatile SpiritStatus g_xStatus;

/**
 * @}
 */



/** @defgroup Types_Private_FunctionPrototypes       Types Private FunctionPrototypes
 * @{
 */



/**
 * @}
 */



/** @defgroup Types_Private_Functions                 Types Private Functions
 * @{
 */

#ifdef  SPIRIT_USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param file  pointer to the source file name
 * @param line  assert_param error line source number
 * @retval : None
 */
void s_assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number */
  printf("Wrong parameters value: file %s on line %d\r\n", file, line);

  /* Infinite loop */
  while (1)
  {
  }
}
#elif SPIRIT_USE_VCOM_ASSERT

#include "SDK_EVAL_VC_General.h"

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param file pointer to the source file name
 * @param line  assert_param error line source number
 * @param expression: string representing the assert failed expression
 * @retval : None
 */
void s_assert_failed(uint8_t* file, uint32_t line, char* expression)
{

  printf("\n\rVCOM DEBUG: Incorrect parameter. Please reboot.\n\r");
  printf("%s:%d \n\r",file,line);
  printf("The expression %s returned FALSE.\n\r", expression);

  /* Infinite loop */
  while (1)
  {
  }
}

#elif SPIRIT_USE_FRAME_ASSERT

#include "SdkUsbProtocol.h"

/**
 * @brief Sends a notify frame with a payload indicating the name 
 *        of the assert failed.
 * @param expression: string representing the assert failed expression
 * @retval : None
 */
void s_assert_failed(char* expression)
{
  char pcPayload[100];
  uint16_t i;
  
  for(i = 0 ; expression[i]!='(' ; i++);
  expression[i]='\0';
  
  strcpy(pcPayload, &expression[3]);
  
  //sprintf(pcPayload, "The expression %s returned FALSE.\n\r", expression);
  SpiritNotifyAssertFailed(pcPayload);

}

#endif


/**
 * @brief  Updates the gState (the global variable used to maintain memory of Spirit Status)
 *         reading the MC_STATE register of SPIRIT.
 * @param  None
 * @retval None
 */
void SpiritRefreshStatus(void)
{
  uint8_t tempRegValue;

  /* Reads the MC_STATUS register to update the g_xStatus */
  g_xStatus = SpiritSpiReadRegisters(MC_STATE1_BASE, 1, &tempRegValue);

}


/**
 * @}
 */



/**
 * @}
 */



/**
 * @}
 */



/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
