/**
 * @file    SPIRIT_Config.h
 * @author  VMA division - AMS
 * @version V2.0.2
 * @date    Febrary 7, 2015
 * @brief   Spirit Configuration and useful defines .
 * @details
 *
 * This file is used to include all or a part of the Spirit
 * libraries into the application program which will be used.
 * Moreover some important parameters are defined here and the
 * user is allowed to edit them.
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
#ifndef __SPIRIT_CONFIG_H
#define __SPIRIT_CONFIG_H


  /* Includes ------------------------------------------------------------------*/
#include "SPIRIT_Regs.h"
#include "SPIRIT_Aes.h"
#include "SPIRIT_Calibration.h"
#include "SPIRIT_Commands.h"
#include "SPIRIT_Csma.h"
#include "SPIRIT_DirectRF.h"
#include "SPIRIT_General.h"
#include "SPIRIT_Gpio.h"
#include "SPIRIT_Irq.h"
#include "SPIRIT_Timer.h"
#include "SPIRIT_LinearFifo.h"
#include "SPIRIT_PktBasic.h"
#include "SPIRIT_PktMbus.h"
#include "SPIRIT_PktStack.h"

#include "SPIRIT_Qi.h"
#include "SPIRIT_Radio.h"
#include "MCU_Interface.h"
#include "SPIRIT_Types.h"
#include "SPIRIT_Management.h"


#ifdef __cplusplus
extern "C" {
#endif


/** @addtogroup SPIRIT_Libraries        SPIRIT Libraries
 * @brief This firmware implements libraries which allow the user
 * to manage the features of Spirit without knowing the hardware details.
 * @details The <i>SPIRIT_Libraries</i> modules are totally platform independent. The library provides one
 * module for each device feature. Each module refers to some functions whose prototypes are located in the
 * header file <i>@ref MCU_Interface.h</i>. The user who want to use these libraries on a particular
 * platform has to implement these functions respecting them signatures.
 * @{
 */

/** @defgroup SPIRIT_Configuration      Configuration
 * @brief Spirit Configuration and useful defines.
 * @details See the file <i>@ref SPIRIT_Config.h</i> for more details.
 * @{
 */


/** @defgroup Configuration_Exported_Types      Configuration Exported Types
 * @{
 */

/**
 * @}
 */


/** @defgroup Configuration_Exported_Constants  Configuration Exported Constants
 * @{
 */


/**
 * @}
 */


/** @defgroup Configuration_Exported_Macros     Configuration Exported Macros
 * @{
 */

/**
 * @}
 */


/** @defgroup Configuration_Exported_Functions  Configuration Exported Functions
 * @{
 */

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
