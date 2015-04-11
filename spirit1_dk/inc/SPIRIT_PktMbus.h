/**
* @file    SPIRIT_PktMbus.h
* @author  VMA division - AMS
* @version 3.2.0
* @date    February 1, 2015
* @brief   Configuration and management of SPIRIT MBUS packets.
* @details
*
* This module can be used to manage the configuration of Spirit MBUS
* packets.
* The user can obtain a packet configuration filling the structure
* <i>@ref PktMbusInit</i>, defining in it some general parameters
* for the Spirit MBUS packet format.
* Since the MBUS protocol is a standard, the configuration of a MBUS
* packet is very simple to do.
*
* <b>Example:</b>
* @code
*
* PktMbusInit mbusInit={
*   MBUS_SUBMODE_S1_S2_LONG_HEADER,    // MBUS submode selection
*   36,                                // added "01" chips on preamble
*   16                                 // postamble length in "01" chips
* };
*
* ...
*
* SpiritPktMbusInit(&mbusInit);
*
* ...
*
* @endcode
*
* The module provides some other functions that can be used to modify
* or read only some configuration parameters.
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
#ifndef __SPIRIT_PACKET_MBUS_H
#define __SPIRIT_PACKET_MBUS_H



/* Includes ------------------------------------------------------------------*/

#include "SPIRIT_Regs.h"
#include "SPIRIT_Types.h"
#include "SPIRIT_PktCommon.h"

#ifdef __cplusplus
 extern "C" {
#endif



/**
 * @addtogroup SPIRIT_Libraries
 * @{
 */


/**
 * @defgroup SPIRIT_PktMbus     Pkt MBUS
 * @brief Configuration and management of SPIRIT MBUS packets.
 * @details See the file <i>@ref SPIRIT_PktMbus.h</i> for more details.
 * @{
 */

/**
 * @defgroup PktMbus_Exported_Types     Pkt MBUS Exported Types
 * @{
 */



/**
 * @brief  MBUS submode enumeration.
 */

typedef enum
{
  MBUS_SUBMODE_S1_S2_LONG_HEADER            = MBUS_CTRL_MBUS_SUBMODE_S1_S2L,                /*!< MBUS submode S1, S2 (long header) - Header length = mbus_prmbl_ctrl + 279 (in "01" bit pairs) , Sync word = 0x7696 (length 18 bits) */
  MBUS_SUBMODE_S1_M_S2_T2_OTHER_TO_METER    = MBUS_CTRL_MBUS_SUBMODE_S2_S1M_T2_OTHER,       /*!< MBUS submode S1-m, S2, T2 (other to meter) - Header length = mbus_prmbl_ctrl + 15 (in "01" bit pairs) , Sync word = 0x7696 (length 18 bits)*/
  MBUS_SUBMODE_T1_T2_METER_TO_OTHER         = MBUS_CTRL_MBUS_SUBMODE_T1_T2_METER,           /*!< MBUS submode T1, T2 (meter to other) - Header length = mbus_prmbl_ctrl + 19 (in "01" bit pairs) ,  Sync word = 0x3D (length 10 bits)*/
  MBUS_SUBMODE_R2_SHORT_HEADER              = MBUS_CTRL_MBUS_SUBMODE_R2,                    /*!< MBUS submode R2, short header - Header length = mbus_prmbl_ctrl + 39 (in "01" bit pairs) , Sync word = 0x7696 (length 18 bits)*/

}MbusSubmode;

#define IS_MBUS_SUBMODE(MODE)   (((MODE) == MBUS_SUBMODE_S1_S2_LONG_HEADER) || \
                                 ((MODE) == MBUS_SUBMODE_S1_M_S2_T2_OTHER_TO_METER) || \
                                 ((MODE) == MBUS_SUBMODE_T1_T2_METER_TO_OTHER) || \
                                 ((MODE) == MBUS_SUBMODE_R2_SHORT_HEADER))


/**
 * @brief  SPIRIT MBUS Packet Init structure definition
 */
typedef struct
{
  MbusSubmode  	    xMbusSubmode;              /*!< Specifies the SUBMODE to be configured.
                                                    This parameter can be a value of @ref MbusSubmode */

  uint8_t           cPreambleLength;           /*!< Specifies the PREAMBLE length.
                                                    This parameter can be any value between 0 and 255 chip sequence '01' */

  uint8_t           cPostambleLength;          /*!< Specifies the POSTAMBLE length.
                                                    This parameter can be any value between 0 and 255 chip sequence '01' */

}PktMbusInit;

/**
 *@}
 */


/**
 * @defgroup PktMbus_Exported_Constants         Pkt MBUS Exported Constants
 * @{
 */


/**
 *@}
 */


/**
 * @defgroup PktMbus_Exported_Macros            Pkt MBUS Exported Macros
 * @{
 */


/**
 *@}
 */


/**
 * @defgroup PktMbus_Exported_Functions                 Pkt MBUS Exported Functions
 * @{
 */
void SpiritPktMbusInit(PktMbusInit* pxPktMbusInit);
void SpiritPktMbusGetInfo(PktMbusInit* pxPktMbusInit);
void SpiritPktMbusSetFormat(void);
void SpiritPktMbusSetPreamble(uint8_t cPreamble);
uint8_t SpiritPktMbusGetPreamble(void);
void SpiritPktMbusSetPostamble(uint8_t cPostamble);
uint8_t SpiritPktMbusGetPostamble(void);
void SpiritPktMbusSetSubmode(MbusSubmode xMbusSubmode);
MbusSubmode SpiritPktMbusGetSubmode(void);
void SpiritPktMbusSetPayloadLength(uint16_t nPayloadLength);
uint16_t SpiritPktMbusGetPayloadLength(void);


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
