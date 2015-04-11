/**
* @file    SPIRIT_Commands.h
* @author  VMA division - AMS
* @version 3.2.0
* @date    February 1, 2015
* @brief   Management of SPIRIT Commands.
* @details
*
* In this module can be found all the API used to strobe commands to
* Spirit.
* Every command strobe is an SPI transaction with a specific command code.
*
* <b>Example:</b>
* @code
*   ...
*
*   SpiritCmdStrobeRx();
*
*   ...
* @endcode
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
#ifndef __SPIRIT_COMMANDS_H
#define __SPIRIT_COMMANDS_H


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
 * @defgroup SPIRIT_Commands    Commands
 * @brief Management of SPIRIT Commands.
 * @details See the file <i>@ref SPIRIT_Commands.h</i> for more details.
 * @{
 */

/**
 * @defgroup Commands_Exported_Types    Commands Exported Types
 * @{
 */

/**
  * @brief  SPIRIT Commands codes enumeration
  */
typedef enum
{
  CMD_TX =  COMMAND_TX,                           /*!< Start to transmit; valid only from READY */
  CMD_RX =  COMMAND_RX,                           /*!< Start to receive; valid only from READY */
  CMD_READY =  COMMAND_READY,                     /*!< Go to READY; valid only from STANDBY or SLEEP or LOCK */
  CMD_STANDBY =  COMMAND_STANDBY,                 /*!< Go to STANDBY; valid only from READY */
  CMD_SLEEP = COMMAND_SLEEP,                      /*!< Go to SLEEP; valid only from READY */
  CMD_LOCKRX = COMMAND_LOCKRX,                    /*!< Go to LOCK state by using the RX configuration of the synth; valid only from READY */
  CMD_LOCKTX = COMMAND_LOCKTX,                    /*!< Go to LOCK state by using the TX configuration of the synth; valid only from READY */
  CMD_SABORT = COMMAND_SABORT,                    /*!< Force exit form TX or RX states and go to READY state; valid only from TX or RX */
  CMD_LDC_RELOAD = COMMAND_LDC_RELOAD,            /*!< LDC Mode: Reload the LDC timer with the value stored in the  LDC_PRESCALER / COUNTER  registers; valid from all states  */
  CMD_SEQUENCE_UPDATE =  COMMAND_SEQUENCE_UPDATE, /*!< Autoretransmission: Reload the Packet sequence counter with the value stored in the PROTOCOL[2] register valid from all states */
  CMD_AES_ENC = COMMAND_AES_ENC,                  /*!< Commands: Start the encryption routine; valid from all states; valid from all states */
  CMD_AES_KEY = COMMAND_AES_KEY,                  /*!< Commands: Start the procedure to compute the key for the decryption; valid from all states */
  CMD_AES_DEC = COMMAND_AES_DEC,                  /*!< Commands: Start the decryption routine using the current key; valid from all states */
  CMD_AES_KEY_DEC = COMMAND_AES_KEY_DEC,          /*!< Commands: Compute the key and start the decryption; valid from all states */
  CMD_SRES = COMMAND_SRES,                        /*!< Reset of all digital part, except SPI registers */
  CMD_FLUSHRXFIFO = COMMAND_FLUSHRXFIFO,          /*!< Clean the RX FIFO; valid from all states */
  CMD_FLUSHTXFIFO = COMMAND_FLUSHTXFIFO,          /*!< Clean the TX FIFO; valid from all states */
} SpiritCmd;

#define IS_SPIRIT_CMD(CMD)  (CMD == CMD_TX || \
                             CMD == CMD_RX || \
                             CMD == CMD_READY || \
                             CMD == CMD_STANDBY || \
                             CMD == CMD_SLEEP || \
                             CMD == CMD_LOCKRX || \
                             CMD == CMD_LOCKTX || \
                             CMD == CMD_SABORT || \
                             CMD == CMD_LDC_RELOAD || \
                             CMD == CMD_SEQUENCE_UPDATE || \
                             CMD == CMD_AES_ENC || \
                             CMD == CMD_AES_KEY || \
                             CMD == CMD_AES_DEC || \
                             CMD == CMD_AES_KEY_DEC || \
                             CMD == CMD_SRES || \
                             CMD == CMD_FLUSHRXFIFO || \
                             CMD == CMD_FLUSHTXFIFO \
                            )

/**
 * @}
 */


/**
 * @defgroup Commands_Exported_Constants        Commands Exported Constants
 * @{
 */


/**
 * @}
 */


/**
 * @defgroup Commands_Exported_Macros   Commands Exported Macros
 * @{
 */

/**
 * @brief  Sends the TX command to SPIRIT. Start to transmit.
 * @param  None.
 * @retval None.
 */
#define SpiritCmdStrobeTx()      {SpiritManagementWaCmdStrobeTx(); \
                                  SpiritCmdStrobeCommand(CMD_TX);} 


/**
 * @brief  Sends the RX command to SPIRIT. Start to receive.
 * @param  None.
 * @retval None.
 */
#define SpiritCmdStrobeRx()      {SpiritManagementWaCmdStrobeRx(); \
                                  SpiritCmdStrobeCommand(CMD_RX); \
                                  } 


/**
 * @brief  Sends the Ready state command to SPIRIT. Go to READY.
 * @param  None.
 * @retval None.
 */
#define SpiritCmdStrobeReady()          SpiritCmdStrobeCommand(CMD_READY)



/**
 * @brief  Sends the Standby command to SPIRIT. Go to STANDBY.
 * @param  None.
 * @retval None.
 */
#define SpiritCmdStrobeStandby()        SpiritCmdStrobeCommand(CMD_STANDBY)



/**
 * @brief  Sends the Sleep command to SPIRIT. Go to SLEEP.
 * @param  None.
 * @retval None.
 */
#define SpiritCmdStrobeSleep()          SpiritCmdStrobeCommand(CMD_SLEEP)



/**
 * @brief  Sends the LOCK_RX command to SPIRIT. Go to the LOCK state by using the RX configuration of the synthesizer.
 * @param  None.
 * @retval None.
 */
#define SpiritCmdStrobeLockRx()         SpiritCmdStrobeCommand(CMD_LOCKRX)



/**
 * @brief  Sends the LOCK_TX command to SPIRIT. Go to the LOCK state by using the TX configuration of the synthesizer.
 * @param  None.
 * @retval None.
 */
#define SpiritCmdStrobeLockTx()         SpiritCmdStrobeCommand(CMD_LOCKTX)



/**
 * @brief  Sends the SABORT command to SPIRIT. Exit from TX or RX states and go to READY state.
 * @param  None.
 * @retval None.
 */
#define SpiritCmdStrobeSabort()         SpiritCmdStrobeCommand(CMD_SABORT)


/**
 * @brief  Sends the LDC_RELOAD command to SPIRIT. Reload the LDC timer with the value stored in the LDC_PRESCALER / COUNTER registers.
 * @param  None.
 * @retval None.
 */
#define SpiritCmdStrobeLdcReload()      SpiritCmdStrobeCommand(CMD_LDC_RELOAD)



/**
 * @brief  Sends the SEQUENCE_UPDATE command to SPIRIT. Reload the Packet sequence counter with the value stored in the PROTOCOL[2] register.
 * @param  None.
 * @retval None.
 */
#define SpiritCmdStrobeSequenceUpdate() SpiritCmdStrobeCommand(CMD_SEQUENCE_UPDATE)



/**
 * @brief  Sends the AES_ENC command to SPIRIT. Starts the encryption routine.
 * @param  None.
 * @retval None.
 */
#define SpiritCmdStrobeAesEnc()         SpiritCmdStrobeCommand(CMD_AES_ENC)



/**
 * @brief  Sends the AES_KEY command to SPIRIT. Starts the procedure to compute the key for the decryption.
 * @param  None.
 * @retval None.
 */
#define SpiritCmdStrobeAesKey()         SpiritCmdStrobeCommand(CMD_AES_KEY)



/**
 * @brief  Sends the AES_DEC command to SPIRIT. Starts the decryption using the current key.
 * @param  None.
 * @retval None.
 */
#define SpiritCmdStrobeAesDec()         SpiritCmdStrobeCommand(CMD_AES_DEC)



/**
 * @brief  Sends the KEY_DEC command to SPIRIT. Computes the key derivation and start the decryption.
 * @param  None.
 * @retval None.
 */
#define SpiritCmdStrobeAesKeyDec()     SpiritCmdStrobeCommand(CMD_AES_KEY_DEC)

/**
 * @brief  Sends the SRES command to SPIRIT. Partial reset: all digital circuit will be reset (exception for SPI only).
 * @param  None.
 * @retval None.
 */
#define SpiritCmdStrobeSres()          SpiritCmdStrobeCommand(CMD_SRES)  
  

/**
 * @brief  Sends the FLUSHRXFIFO command to SPIRIT. Clean the RX FIFO.
 * @param  None.
 * @retval None.
 */
#define SpiritCmdStrobeFlushRxFifo()    SpiritCmdStrobeCommand(CMD_FLUSHRXFIFO)



/**
 * @brief  Sends the FLUSHTXFIFO command to SPIRIT. Clean the TX FIFO.
 * @param  None.
 * @retval None.
 */
#define SpiritCmdStrobeFlushTxFifo()    SpiritCmdStrobeCommand(CMD_FLUSHTXFIFO)



/**
 * @}
 */


/**
 * @defgroup Commands_Exported_Functions    Commands Exported Functions
 * @{
 */
void SpiritCmdStrobeCommand(SpiritCmd xCommandCode);


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
