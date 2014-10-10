/**
* @file    SPIRIT_Management.c
 * @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
 * @version V3.0.1
 * @date    November 19, 2012
 * @brief   The management layer for SPIRIT1 library.
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
* <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
*/  


/* Includes ------------------------------------------------------------------*/
#include "SPIRIT_Management.h"

/**
 * @addtogroup SPIRIT_Libraries
 * @{
 */


/**
* @defgroup SPIRIT_MANAGEMENT              SPIRIT Management
* @{
*/

/**
* @brief  BS value to write in the SYNT0 register according to the selected band
*/
static const uint8_t s_vectcBandRegValue[4]={SYNT0_BS_6, SYNT0_BS_12, SYNT0_BS_16, SYNT0_BS_32};

#define COMMUNICATION_STATE_TX          0
#define COMMUNICATION_STATE_RX          1
#define COMMUNICATION_STATE_NONE        2

static uint32_t s_nDesiredFrequency;

volatile static uint8_t s_cCommunicationState = COMMUNICATION_STATE_NONE;


/**
* @brief  Factor is: B/2 used in the formula for SYNTH word calculation
*/
static const uint8_t s_vectcBHalfFactor[4]={(HIGH_BAND_FACTOR/2), (MIDDLE_BAND_FACTOR/2), (LOW_BAND_FACTOR/2), (VERY_LOW_BAND_FACTOR/2)};


/**
* @defgroup SPIRIT_MANAGEMENT_FUNCTIONS    SPIRIT Management Functions
* @{
*/


/**
* @defgroup WORKAROUND_FUNCTIONS              SPIRIT Management Workaround Functions
* @{
*/

/**
* @brief  Private SpiritRadioSetFrequencyBase function only used in SpiritManagementWaVcoCalibration.
* @param  lFBase the base carrier frequency expressed in Hz as unsigned word.
* @retval None.
*/
void SpiritManagementSetFrequencyBase(uint32_t lFBase)
{
  uint32_t synthWord, Fc;
  uint8_t band, anaRadioRegArray[4], wcp;
  
  /* Check the parameter */
  s_assert_param(IS_FREQUENCY_BAND(lFBase));
  
  /* Search the operating band */
  if(IS_FREQUENCY_BAND_HIGH(lFBase))
  {
    band = HIGH_BAND;
  }
  else if(IS_FREQUENCY_BAND_MIDDLE(lFBase))
  {
    band = MIDDLE_BAND;
  }
  else if(IS_FREQUENCY_BAND_LOW(lFBase))
  {
    band = LOW_BAND;
  }
  else if(IS_FREQUENCY_BAND_VERY_LOW(lFBase))
  {
    band = VERY_LOW_BAND;
  }
  else
  {
    band = 0;
    SpiritReportError(SPI1_WRONG_BAND);
  }
  
  int32_t FOffset  = SpiritRadioGetFrequencyOffset();
  uint32_t lChannelSpace  = SpiritRadioGetChannelSpace();
  uint8_t cChannelNum = SpiritRadioGetChannel();
  
  /* Calculates the channel center frequency */
  Fc = lFBase + FOffset + lChannelSpace*cChannelNum;
  
  /* Reads the reference divider */
  uint8_t cRefDiv = (uint8_t)SpiritRadioGetRefDiv()+1;
  
  if(SpiritGeneralGetSpiritVersion() != SPIRIT_VERSION_2_0) 
  {
    switch(band)
    {
    case VERY_LOW_BAND:
      if(Fc<161281250)
      {
        SpiritCalibrationSelectVco(VCO_L);
      }
      else
      {
        SpiritCalibrationSelectVco(VCO_H);
      }
      break;
      
    case LOW_BAND:
      if(Fc<322562500)
      {
        SpiritCalibrationSelectVco(VCO_L);
      }
      else
      {
        SpiritCalibrationSelectVco(VCO_H);
      }
      break;
      
    case MIDDLE_BAND:
      if(Fc<430083334)
      {
        SpiritCalibrationSelectVco(VCO_L);
      }
      else
      {
        SpiritCalibrationSelectVco(VCO_H);
      }
      break;
      
    case HIGH_BAND:
      if(Fc<860166667)
      {
        SpiritCalibrationSelectVco(VCO_L);
      }
      else
      {
        SpiritCalibrationSelectVco(VCO_H);
      }
    }
  }

  /* Search the VCO charge pump word and set the corresponding register */
  wcp = SpiritRadioSearchWCP(Fc);

  synthWord = (uint32_t)(lFBase*(((double)(FBASE_DIVIDER*cRefDiv*s_vectcBHalfFactor[band]))/SpiritRadioGetXtalFrequency()));
  
  /* Build the array of registers values for the analog part */
  anaRadioRegArray[0] = (uint8_t)(((synthWord>>21)&(0x0000001F))|(wcp<<5));
  anaRadioRegArray[1] = (uint8_t)((synthWord>>13)&(0x000000FF));
  anaRadioRegArray[2] = (uint8_t)((synthWord>>5)&(0x000000FF));
  anaRadioRegArray[3] = (uint8_t)(((synthWord&0x0000001F)<<3)| s_vectcBandRegValue[band]);
  
  /* Configures the needed Analog Radio registers */
  g_xStatus = SpiritSpiWriteRegisters(SYNT3_BASE, 4, anaRadioRegArray);
}

void SpiritManagementWaVcoCalibration(void)
{
  uint8_t s_cVcoWordRx;
  uint8_t s_cVcoWordTx;
  uint32_t nFreq;
  uint8_t cRestore = 0;
  uint8_t cStandby = 0;
  uint32_t xtal_frequency = SpiritRadioGetXtalFrequency();
  SpiritVersion spirit_version = SpiritGeneralGetSpiritVersion();
  
  /* Enable the reference divider if the XTAL is between 48 and 52 MHz */
  if(xtal_frequency>26000000)
  {
    if(!SpiritRadioGetRefDiv())
    {
      cRestore = 1;
      nFreq = SpiritRadioGetFrequencyBase();
      SpiritRadioSetRefDiv(S_ENABLE);
      SpiritManagementSetFrequencyBase(nFreq);
    }
  }
  nFreq = SpiritRadioGetFrequencyBase();
  
  /* Increase the VCO current */
  uint8_t tmp = 0x19; SpiritSpiWriteRegisters(0xA1,1,&tmp);
  
  SpiritCalibrationVco(S_ENABLE);
  
  SpiritRefreshStatus();
  if(g_xStatus.MC_STATE == MC_STATE_STANDBY)
  {
    cStandby = 1;
    SpiritCmdStrobeReady();
    do{
      SpiritRefreshStatus();
    }while(g_xStatus.MC_STATE != MC_STATE_READY); 
  }
  
  SpiritCmdStrobeLockTx();
  
  do{
    SpiritRefreshStatus();
  }while(g_xStatus.MC_STATE != MC_STATE_LOCK);
  
  s_cVcoWordTx = SpiritCalibrationGetVcoCalData();
  
  SpiritCmdStrobeReady();
  
  do{
    SpiritRefreshStatus();
  }while(g_xStatus.MC_STATE != MC_STATE_READY); 
  
  /* Enable the reference divider if the XTAL is between 48 and 52 MHz */
  if(xtal_frequency>26000000 && spirit_version == SPIRIT_VERSION_2_1)
  {
    SpiritManagementSetFrequencyBase(nFreq+480300);
  }
  
  SpiritCmdStrobeLockRx();
  
  do{
    SpiritRefreshStatus();
  }while(g_xStatus.MC_STATE != MC_STATE_LOCK);
  
  s_cVcoWordRx = SpiritCalibrationGetVcoCalData();
  
  SpiritCmdStrobeReady();
  
  do{
    SpiritRefreshStatus();
  }while(g_xStatus.MC_STATE != MC_STATE_READY);
  
  if(cStandby == 1)
  {
    SpiritCmdStrobeStandby();    
  }
  SpiritCalibrationVco(S_DISABLE);
  
  /* Disable the reference divider if the XTAL is between 48 and 52 MHz */
  if(cRestore || (xtal_frequency>26000000 && spirit_version == SPIRIT_VERSION_2_1))
  {
    SpiritRadioSetRefDiv(S_DISABLE);    
    SpiritManagementSetFrequencyBase(nFreq);
  }
  
  /* Restore the VCO current */
  tmp = 0x11; SpiritSpiWriteRegisters(0xA1,1,&tmp);
  
  SpiritCalibrationSetVcoCalDataTx(s_cVcoWordTx);
  SpiritCalibrationSetVcoCalDataRx(s_cVcoWordRx);
}

void SpiritManagementWaRcoCalibration(void)
{
  uint32_t xtal_frequency = SpiritRadioGetXtalFrequency();
  SpiritVersion spirit_version = SpiritGeneralGetSpiritVersion();
  
  if(spirit_version == SPIRIT_VERSION_2_1 && xtal_frequency<48000000)
  {
    uint8_t tmp= 0x02;SpiritSpiWriteRegisters(0x50, 1, &tmp); 
    SpiritCmdStrobeStandby(); 
    tmp= 0x29;SpiritSpiWriteRegisters(0xB4, 1, &tmp); 
    tmp= 0x06;SpiritSpiWriteRegisters(0x50, 1, &tmp); 
    SpiritCmdStrobeReady();
  }
}


void SpiritManagementWaRxStartup(void)
{
  SpiritVersion spirit_version = SpiritGeneralGetSpiritVersion();

  if(spirit_version == SPIRIT_VERSION_2_1)
  {
    for(volatile uint32_t i=0;i<0xFF;i++);
    uint8_t tmp = 0x10; SpiritSpiWriteRegisters(0xa8, 1, &tmp);
    tmp = 0x00; SpiritSpiWriteRegisters(0xa8, 1, &tmp);
    {
      //      uint8_t tmp1,tmp2 = 0x5F; SpiritSpiReadRegisters(0x9E, 1, &tmp1);
      //      SpiritCmdStrobeCommand(CMD_RX); 
      //      SpiritSpiWriteRegisters(0x9E, 1, &tmp2);
      //      for(volatile uint32_t i=0;i<0x3F;i++);
      //      SpiritSpiWriteRegisters(0x9E, 1, &tmp1);
    }
  }  
}


void SpiritManagementWaRxStartupInit(void)
{
  SpiritVersion spirit_version = SpiritGeneralGetSpiritVersion();
  
  if(spirit_version == SPIRIT_VERSION_2_1)
  {
      uint8_t tmp = 0xA8; SpiritSpiWriteRegisters(0xa9, 1, &tmp);
      tmp = 0x4B; SpiritSpiWriteRegisters(0xaa, 1, &tmp);
      tmp = 0xFC; SpiritSpiWriteRegisters(0xab, 1, &tmp);
  }  
}


void SpiritManagementWaCmdStrobeTx(void)
{
  if(s_cCommunicationState != COMMUNICATION_STATE_TX)
  {
    uint32_t xtal_frequency = SpiritRadioGetXtalFrequency();
    SpiritVersion spirit_version = SpiritGeneralGetSpiritVersion();
    
    if(spirit_version == SPIRIT_VERSION_2_1  && xtal_frequency>26000000)
    {
      SpiritManagementSetFrequencyBase(s_nDesiredFrequency);      
    }
    /* To achive the max output power */
    if(s_nDesiredFrequency>=150000000 && s_nDesiredFrequency<=470000000)
    {
      /* Optimal setting for Tx mode only */
      SpiritRadioSetPACwc(LOAD_3_6_PF);
    }
    else
    {
      if(spirit_version == SPIRIT_VERSION_3_0_D1 && s_nDesiredFrequency>=863000000 && s_nDesiredFrequency<=870000000) {
        /* Optimal setting for Tx mode only */
        SpiritRadioSetPACwc(LOAD_2_4_PF);
      }
      else {
        /* Optimal setting for Tx mode only */
        SpiritRadioSetPACwc(LOAD_0_PF);
      }
    }
    
    uint8_t tmp = 0x11; SpiritSpiWriteRegisters(0xa9, 1, &tmp); /* Enable VCO_L buffer */
    tmp = 0x20; SpiritSpiWriteRegisters(PM_CONFIG1_BASE, 1, &tmp); /* Set SMPS switching frequency */
    
    s_cCommunicationState = COMMUNICATION_STATE_TX;
  }
}


void SpiritManagementWaCmdStrobeRx(void)
{
  if(s_cCommunicationState != COMMUNICATION_STATE_RX)
  {
    uint32_t xtal_frequency = SpiritRadioGetXtalFrequency();
    SpiritVersion spirit_version = SpiritGeneralGetSpiritVersion();
    
    if(spirit_version == SPIRIT_VERSION_2_1 && xtal_frequency>26000000)
    {
      SpiritManagementSetFrequencyBase(s_nDesiredFrequency+480300);
      SpiritManagementWaRxStartupInit();
    }
    
    uint8_t tmp = 0x90; SpiritSpiWriteRegisters(PM_CONFIG1_BASE, 1, &tmp); /* Set SMPS switching frequency */    
    SpiritRadioSetPACwc(LOAD_0_PF); /* Set the correct CWC parameter */
    
    s_cCommunicationState = COMMUNICATION_STATE_RX;
  }
}

void SpiritManagementWaTRxFcMem(uint32_t nDesiredFreq)
{
  s_cCommunicationState = COMMUNICATION_STATE_NONE;
  s_nDesiredFrequency = nDesiredFreq;
}



void SpiritManagementWaRcoRangeExtCalibration(void)
{
  uint32_t xtal_frequency = SpiritRadioGetXtalFrequency();
  SpiritVersion spirit_version = SpiritGeneralGetSpiritVersion();
  
  if(spirit_version == SPIRIT_VERSION_2_1 && xtal_frequency<48000000)
  {
    uint8_t tmp= 0x02;SpiritSpiWriteRegisters(0x50, 1, &tmp); 
    SpiritCmdStrobeStandby(); 
    tmp= 0x29;SpiritSpiWriteRegisters(0xB4, 1, &tmp); 
    tmp= 0x06;SpiritSpiWriteRegisters(0x50, 1, &tmp); 
    SpiritCmdStrobeReady();
  }
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

/**
* @}
*/


/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
