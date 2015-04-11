/**
* @file    SPIRIT_Calibration.h
* @author  VMA division - AMS
* @version 3.2.0
* @date    February 1, 2015
* @brief   Configuration and management of SPIRIT VCO-RCO calibration.
* @details
*
* This module allows the user to set some parameters which deal
* with the oscillators calibration.
* The state machine of Spirit contemplates some optional calibrating operations
* in the transition between the READY and the LOCK state.
* The user is allowed to enable or disable the automatic RCO/VCO calibration
* by calling the functions <i>@ref SpiritCalibrationVco()</i> and <i>@ref SpiritCalibrationRco()</i>.
* The following example shows how to do an initial calibration of VCO.
*
* <b>Example:</b>
* @code
*  uint8_t calData;
*
*  SpiritCalibrationVco(S_ENABLE);
*  SpiritCmdStrobeLockTx();
*
*  while(g_xStatus.MC_STATE != MC_STATE_LOCK){
*      SpiritRefreshStatus();
*  }
*
*  calData = SpiritCalibrationGetVcoCalDataTx();
*  SpiritCalibrationSetVcoCalDataTx(calData);
*
*  SpiritCmdStrobeReady();
*  SpiritCalibrationVco(S_DISABLE);
*
* @endcode
*
* Similar operations can be done for the RCO calibrator.
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
#ifndef __SPIRIT_CALIBRATION_H
#define __SPIRIT_CALIBRATION_H


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
 * @defgroup SPIRIT_Calibration Calibration
 * @brief Configuration and management of SPIRIT VCO-RCO calibration.
 * @details See the file <i>@ref SPIRIT_Calibration.h</i> for more details.
 * @{
 */

/**
 * @defgroup Calibration_Exported_Types Calibration Exported Types
 * @{
 */


/**
 * @brief  VCO / RCO calibration window.
 */
typedef enum
{

  CALIB_TIME_7_33_US_24MHZ = 0x00,	/*!< calibration window of 7.33 us with XTAL=24MHz */
  CALIB_TIME_14_67_US_24MHZ,		/*!< calibration window of 14.67 us with XTAL=24MHz */
  CALIB_TIME_29_33_US_24MHZ,		/*!< calibration window of 29.33 us with XTAL=24MHz */
  CALIB_TIME_58_67_US_24MHZ,		/*!< calibration window of 58.67 us with XTAL=24MHz */

  CALIB_TIME_6_77_US_26MHZ = 0x00,	/*!< calibration window of 6.77 us with XTAL=26MHz */
  CALIB_TIME_13_54_US_26MHZ,		/*!< calibration window of 13.54 us with XTAL=26MHz */
  CALIB_TIME_27_08_US_26MHZ,		/*!< calibration window of 27.08 us with XTAL=26MHz */
  CALIB_TIME_54_15_US_26MHZ		/*!< calibration window of 54.15 us with XTAL=26MHz */

} VcoWin;


#define IS_VCO_WIN(REF)   (REF == CALIB_TIME_7_33_US_24MHZ  ||\
                           REF == CALIB_TIME_14_67_US_24MHZ ||\
                           REF == CALIB_TIME_29_33_US_24MHZ ||\
                           REF == CALIB_TIME_58_67_US_24MHZ ||\
                           REF == CALIB_TIME_6_77_US_26MHZ  ||\
                           REF == CALIB_TIME_13_54_US_26MHZ ||\
                           REF == CALIB_TIME_27_08_US_26MHZ ||\
                           REF == CALIB_TIME_54_15_US_26MHZ \
                           )

/**
 * @brief  VCO_H / VCO_L selection.
 */
typedef enum
{

  VCO_L = 0x00,	        /*!< VCO lower */
  VCO_H,		/*!< VCO higher */
} VcoSel;


#define IS_VCO_SEL(REF)   (REF == VCO_L  ||\
                           REF == VCO_H \
                           )


/**
 * @}
 */


/**
 * @defgroup Calibration_Exported_Constants     Calibration Exported Constants
 * @{
 */

/**
 * @}
 */



/** @defgroup VCO_Calibration   VCO Calibration
 * @{
 */

/**
 * @}
 */




/**
 * @defgroup Calibration_Exported_Macros        Calibration Exported Macros
 * @{
 */


/**
 * @}
 */


/**
 * @defgroup Calibration_Exported_Functions     Calibration Exported Functions
 * @{
 */

void SpiritCalibrationRco(SpiritFunctionalState xNewState);
void SpiritCalibrationVco(SpiritFunctionalState xNewState);
void SpiritCalibrationSetRcoCalWords(uint8_t cRwt, uint8_t cRfb);
void SpiritCalibrationGetRcoCalWords(uint8_t* pcRwt, uint8_t* pcRfb);
uint8_t SpiritCalibrationGetVcoCalData(void);
void SpiritCalibrationSetVcoCalDataTx(uint8_t cVcoCalData);
uint8_t SpiritCalibrationGetVcoCalDataTx(void);
void SpiritCalibrationSetVcoCalDataRx(uint8_t cVcoCalData);
uint8_t SpiritCalibrationGetVcoCalDataRx(void);
void SpiritCalibrationSetVcoWindow(VcoWin xRefWord);
VcoWin SpiritCalibrationGetVcoWindow(void);
VcoSel SpiritCalibrationGetVcoSelecttion(void);
void SpiritCalibrationSelectVco(VcoSel xVco);

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
