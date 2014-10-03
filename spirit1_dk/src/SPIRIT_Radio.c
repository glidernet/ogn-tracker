/**
* @file    SPIRIT_Radio.c
* @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
* @version V3.0.1
* @date    November 19, 2012
* @brief   This file provides all the low level API to manage Analog and Digital
*          radio part of SPIRIT.
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
*
*/

/* Includes ------------------------------------------------------------------*/
#include "SPIRIT_Radio.h"
#include "MCU_Interface.h"
#include <math.h>

/** @addtogroup SPIRIT_Libraries
* @{
*/


/** @addtogroup SPIRIT_Radio
* @{
*/


/** @defgroup Radio_Private_TypesDefinitions            Radio Private Types Definitions
* @{
*/


/**
* @}
*/


/** @defgroup Radio_Private_Defines                     Radio Private Defines
* @{
*/




/**
* @}
*/


/** @defgroup Radio_Private_Macros                      Radio Private Macros
* @{
*/
#define XTAL_FLAG(xtalFrequency)               xtalFrequency>=25e6 ? XTAL_FLAG_26_MHz:XTAL_FLAG_24_MHz

/**
* @}
*/


/** @defgroup Radio_Private_Variables                   Radio Private Variables
* @{
*/
/**
* @brief  The Xtal frequency. To be set by the user (see SetXtalFreq() function)
*/
static uint32_t s_lXtalFrequency;

/**
* @brief  Factor is: B/2 used in the formula for SYNTH word calculation
*/
static const uint8_t s_vectcBHalfFactor[4]={(HIGH_BAND_FACTOR/2), (MIDDLE_BAND_FACTOR/2), (LOW_BAND_FACTOR/2), (VERY_LOW_BAND_FACTOR/2)};

/**
* @brief  BS value to write in the SYNT0 register according to the selected band
*/
static const uint8_t s_vectcBandRegValue[4]={SYNT0_BS_6, SYNT0_BS_12, SYNT0_BS_16, SYNT0_BS_32};


/**
* @brief  It represents the available channel bandwidth times 10 for 26 Mhz xtal.
* @note   The channel bandwidth for others xtal frequencies can be computed since this table
*         multiplying the current table by a factor xtal_frequency/26e6.
*/
static const uint16_t s_vectnBandwidth26M[90]=
{
  8001, 7951, 7684, 7368, 7051, 6709, 6423, 5867, 5414, \
    4509, 4259, 4032, 3808, 3621, 3417, 3254, 2945, 2703, \
      2247, 2124, 2011, 1900, 1807, 1706, 1624, 1471, 1350, \
        1123, 1062, 1005,  950,  903,  853,  812,  735,  675, \
          561,  530,  502,  474,  451,  426,  406,  367,  337, \
            280,  265,  251,  237,  226,  213,  203,  184,  169, \
              140,  133,  126,  119,  113,  106,  101,   92,   84, \
                70,   66,   63,   59,   56,   53,   51,   46,   42, \
                  35,   33,   31,   30,   28,   27,   25,   23,   21, \
                    18,   17,   16,   15,   14,   13,   13,   12,   11
};

/**
* @brief  It represents the available VCO frequencies
*/
static const uint16_t s_vectnVCOFreq[16]=
{
  4644, 4708, 4772, 4836, 4902, 4966, 5030, 5095, \
    5161, 5232, 5303, 5375, 5448, 5519, 5592, 5663
};


/**
* @}
*/


/** @defgroup Radio_Private_FunctionPrototypes          Radio Private Function Prototypes
* @{
*/


/**
* @}
*/


/** @defgroup Radio_Private_Functions                    Radio Private Functions
* @{
*/

/**
* @brief  Initializes the SPIRIT analog and digital radio part according to the specified
*         parameters in the pxSRadioInitStruct.
* @param  pxSRadioInitStruct pointer to a SRadioInit structure that
*         contains the configuration information for the analog radio part of SPIRIT.
* @retval None.
*/
void SpiritRadioInit(SRadioInit* pxSRadioInitStruct)
{
  int32_t FOffsetTmp;
  uint8_t anaRadioRegArray[8], digRadioRegArray[4];
  int16_t xtalOffsetFactor;
  uint8_t drM, drE, FdevM, FdevE, bwM, bwE;
  
  /* Get the RF board version */
  SpiritVersion xSpiritVersion = SpiritGeneralGetSpiritVersion();
  
  /* Workaround for Vtune */
  uint8_t value = 0xA0; SpiritSpiWriteRegisters(0x9F, 1, &value);
  
  /* Calculates the offset respect to RF frequency and according to xtal_ppm parameter: (xtal_ppm*FBase)/10^6 */
  FOffsetTmp = (int32_t)((pxSRadioInitStruct->nXtalOffsetPpm*pxSRadioInitStruct->lFrequencyBase)/PPM_FACTOR);
  
  /* Check the parameters */
  s_assert_param(IS_FREQUENCY_BAND(pxSRadioInitStruct->lFrequencyBase));
  s_assert_param(IS_MODULATION_SELECTED(pxSRadioInitStruct->xModulationSelect));
  s_assert_param(IS_DATARATE(pxSRadioInitStruct->lDatarate));
  s_assert_param(IS_FREQUENCY_OFFSET(FOffsetTmp,s_lXtalFrequency));
  s_assert_param(IS_CHANNEL_SPACE(pxSRadioInitStruct->nChannelSpace,s_lXtalFrequency));
  s_assert_param(IS_F_DEV(pxSRadioInitStruct->lFreqDev,s_lXtalFrequency));
  
  /* Disable the digital, ADC, SMPS reference clock divider if fXO>24MHz or fXO<26MHz */
  if(xSpiritVersion != SPIRIT_VERSION_2_0)
  {
    /* Goes in STANDBY state */
    SpiritSpiCommandStrobes(COMMAND_STANDBY);    
    do{
      /* Delay for state transition */
      for(volatile uint8_t i=0; i!=0xFF; i++);
      
      /* Reads the MC_STATUS register */
      SpiritRefreshStatus();
    }while(g_xStatus.MC_STATE!=MC_STATE_STANDBY);
    
    if(s_lXtalFrequency<48000000)
    {
      SpiritRadioSetDigDiv(S_DISABLE);
      s_assert_param(IS_CH_BW(pxSRadioInitStruct->lBandwidth,s_lXtalFrequency));
    }
    else
    {      
      SpiritRadioSetDigDiv(S_ENABLE);
      s_assert_param(IS_CH_BW(pxSRadioInitStruct->lBandwidth,(s_lXtalFrequency>>1)));
    }

    /* Goes in READY state */
    SpiritSpiCommandStrobes(COMMAND_READY);
    do{
      /* Delay for state transition */
      for(volatile uint8_t i=0; i!=0xFF; i++);
      
      /* Reads the MC_STATUS register */
     SpiritRefreshStatus();
    }while(g_xStatus.MC_STATE!=MC_STATE_READY);
  }
  
  /* Calculates the FC_OFFSET parameter and cast as signed int: FOffsetTmp = (Fxtal/2^18)*FC_OFFSET */
  xtalOffsetFactor = (FOffsetTmp*FBASE_DIVIDER)/s_lXtalFrequency;
  anaRadioRegArray[2] = (uint8_t)((((uint16_t)xtalOffsetFactor)>>8)&0x0F);
  anaRadioRegArray[3] = (uint8_t)(xtalOffsetFactor);
  
  /* Calculates the channel space factor */
  anaRadioRegArray[0] =((uint32_t)pxSRadioInitStruct->nChannelSpace<<9)/(s_lXtalFrequency>>6)+1;
  
  SpiritManagementWaTRxFcMem(pxSRadioInitStruct->lFrequencyBase);
  
  /* 2nd order DEM algorithm enabling */
  uint8_t tmpreg; SpiritSpiReadRegisters(0xA3, 1, &tmpreg);
  tmpreg &= ~0x02; SpiritSpiWriteRegisters(0xA3, 1, &tmpreg);
  
  /* Check the channel center frequency is in one of the possible range */
  s_assert_param(IS_FREQUENCY_BAND((pxSRadioInitStruct->lFrequencyBase + ((xtalOffsetFactor*s_lXtalFrequency)/FBASE_DIVIDER) + pxSRadioInitStruct->nChannelSpace * pxSRadioInitStruct->cChannelNumber)));  
  
  /* Calculates the datarate mantissa and exponent */
  SpiritRadioSearchDatarateME(pxSRadioInitStruct->lDatarate, &drM, &drE);
  digRadioRegArray[0] = (uint8_t)(drM);
  digRadioRegArray[1] = (uint8_t)(0x00 | pxSRadioInitStruct->xModulationSelect |drE);
  
  /* Calculates the frequency deviation mantissa and exponent */
  SpiritRadioSearchFreqDevME(pxSRadioInitStruct->lFreqDev, &FdevM, &FdevE);
  digRadioRegArray[2] = (uint8_t)((FdevE<<4) | 0x00 | FdevM);
  
  /* Calculates the channel filter mantissa and exponent */
  SpiritRadioSearchChannelBwME(pxSRadioInitStruct->lBandwidth, &bwM, &bwE);
  
  digRadioRegArray[3] = (uint8_t)((bwM<<4) | bwE);
  
  if(xSpiritVersion == SPIRIT_VERSION_3_0 || xSpiritVersion == SPIRIT_VERSION_3_0_D1)
  {
    uint8_t ifOffsetAna;
    if(s_lXtalFrequency==24000000) {
      ifOffsetAna = 0xB6;
      anaRadioRegArray[1] = 0xB6;
    }
    if(s_lXtalFrequency==25000000) {
      ifOffsetAna = 0xAC;
      anaRadioRegArray[1] = 0xAC;
    }
    if(s_lXtalFrequency==26000000) {
      ifOffsetAna = 0xA3;
      anaRadioRegArray[1] = 0xA3;
    }
    if(s_lXtalFrequency==48000000) {
      ifOffsetAna = 0x3B;
      anaRadioRegArray[1] = 0xB6;
    }
    if(s_lXtalFrequency==50000000) {
      ifOffsetAna = 0x36;
      anaRadioRegArray[1] = 0xAC;
    }
    if(s_lXtalFrequency==52000000) {
      ifOffsetAna = 0x31;
      anaRadioRegArray[1] = 0xA3;
    }
    
    g_xStatus = SpiritSpiWriteRegisters(IF_OFFSET_ANA_BASE, 1, &ifOffsetAna);
  }
  else
  {
    if(s_lXtalFrequency==24000000 || s_lXtalFrequency==48000000) {
      anaRadioRegArray[1] = 0xB6;
    }
    if(s_lXtalFrequency==25000000 || s_lXtalFrequency==50000000) {
      anaRadioRegArray[1] = 0xAC;
    }
    if(s_lXtalFrequency==26000000 || s_lXtalFrequency==52000000) {
      anaRadioRegArray[1] = 0xA3;
    }
  }
  
  /* Sets Xtal configuration */
  if(s_lXtalFrequency>26000000) 
  {
    SpiritRadioSetXtalFlag(XTAL_FLAG((s_lXtalFrequency/2)));
  }
  else
  {
    SpiritRadioSetXtalFlag(XTAL_FLAG(s_lXtalFrequency));
  }
  
  /* Sets the channel number in the corresponding register */
  SpiritSpiWriteRegisters(CHNUM_BASE, 1, &pxSRadioInitStruct->cChannelNumber);
  
  /* Configures the Analog Radio registers */
  SpiritSpiWriteRegisters(CHSPACE_BASE, 4, anaRadioRegArray);
  
  /* Configures the Digital Radio registers */
  g_xStatus = SpiritSpiWriteRegisters(MOD1_BASE, 4, digRadioRegArray);
  
  /* Enable the freeze option of the AFC on the SYNC word */
  SpiritRadioAFCFreezeOnSync(S_ENABLE);
  
  SpiritRadioSetFrequencyBase(pxSRadioInitStruct->lFrequencyBase);
  
}


/**
* @brief  Returns the SPIRIT analog and digital radio structure according to the registers value.
* @param  pxSRadioInitStruct pointer to a SRadioInit structure that
*         contains the configuration information for the analog radio part of SPIRIT.
* @retval None.
*/
void SpiritRadioGetInfo(SRadioInit* pxSRadioInitStruct)
{
  uint8_t anaRadioRegArray[8], digRadioRegArray[4];
  BandSelect band;
  int16_t xtalOffsetFactor;
  
  /* Get the RF board version */
  SpiritVersion xSpiritVersion = SpiritGeneralGetSpiritVersion();
  
  /* Reads the Analog Radio registers */
  SpiritSpiReadRegisters(SYNT3_BASE, 8, anaRadioRegArray);
  
  /* Reads the Digital Radio registers */
  g_xStatus = SpiritSpiReadRegisters(MOD1_BASE, 4, digRadioRegArray);
  
  /* Reads the operating band masking the Band selected field */
  if((anaRadioRegArray[3] & 0x07) == SYNT0_BS_6)
  {
    band = HIGH_BAND;
  }
  else if ((anaRadioRegArray[3] & 0x07) == SYNT0_BS_12)
  {
    band = MIDDLE_BAND;
  }
  else if ((anaRadioRegArray[3] & 0x07) == SYNT0_BS_16)
  {
    band = LOW_BAND;
  }
  else if ((anaRadioRegArray[3] & 0x07) == SYNT0_BS_32)
  {
    band = VERY_LOW_BAND;
  }
  
  /* Computes the synth word */
  uint32_t synthWord = (uint32_t)((((uint32_t)(anaRadioRegArray[0]&0x1F))<<21)+(((uint32_t)(anaRadioRegArray[1]))<<13)+\
    (((uint32_t)(anaRadioRegArray[2]))<<5)+(((uint32_t)(anaRadioRegArray[3]))>>3));
  
  /* Calculates the frequency base */
  uint8_t cRefDiv = (uint8_t)SpiritRadioGetRefDiv()+1;
  pxSRadioInitStruct->lFrequencyBase = (uint32_t)round(synthWord*(((double)s_lXtalFrequency)/(FBASE_DIVIDER*cRefDiv*s_vectcBHalfFactor[band])));
  
  /* Calculates the Offset Factor */
  uint16_t xtalOffTemp = ((((uint16_t)anaRadioRegArray[6])<<8)+((uint16_t)anaRadioRegArray[7]));
  
  /* If a negative number then convert the 12 bit 2-complement in a 16 bit number */
  if(xtalOffTemp & 0x0800)
  {
    xtalOffTemp = xtalOffTemp | 0xF000;
  }
  else
  {
    xtalOffTemp = xtalOffTemp & 0x0FFF;
  }
  
  xtalOffsetFactor = *((int16_t*)(&xtalOffTemp));
  
  /* Calculates the frequency offset in ppm */
  pxSRadioInitStruct->nXtalOffsetPpm =(int16_t)((uint32_t)xtalOffsetFactor*s_lXtalFrequency*PPM_FACTOR)/((uint32_t)FBASE_DIVIDER*pxSRadioInitStruct->lFrequencyBase);
  
  /* Channel space */
  pxSRadioInitStruct->nChannelSpace = anaRadioRegArray[4]*(s_lXtalFrequency>>15);
  
  /* Channel number */
  pxSRadioInitStruct->cChannelNumber = SpiritRadioGetChannel();
  
  /* Modulation select */
  pxSRadioInitStruct->xModulationSelect = (ModulationSelect)(digRadioRegArray[1] & 0x70);
  
  /* Reads the frequency deviation for mantissa and exponent */
  uint8_t FDevM = digRadioRegArray[2]&0x07;
  uint8_t FDevE = (digRadioRegArray[2]&0xF0)>>4;
  
  /* Reads the channel filter register for mantissa and exponent */
  uint8_t bwM = (digRadioRegArray[3]&0xF0)>>4;
  uint8_t bwE = digRadioRegArray[3]&0x0F;
  
  uint8_t cDivider = 0;
  uint8_t cCorrection = 0;
  if(xSpiritVersion != SPIRIT_VERSION_2_0)
  {  
    cDivider = SpiritRadioGetDigDiv();
    cCorrection = 1;
  }
  
  /* Calculates the datarate */
  pxSRadioInitStruct->lDatarate = ((s_lXtalFrequency>>(5+cDivider))*(256+digRadioRegArray[0]))>>(23-(digRadioRegArray[1]&0x0F));
  
  /* Calculates the frequency deviation */
  pxSRadioInitStruct->lFreqDev = (((s_lXtalFrequency>>6)*(8+FDevM))>>(12-FDevE+cCorrection));
  
  /* Reads the channel filter bandwidth from the look-up table and return it */
  pxSRadioInitStruct->lBandwidth = (uint32_t)(100.0*s_vectnBandwidth26M[bwM+(bwE*9)]*((s_lXtalFrequency>>cDivider)/26e6));
  
}


/**
* @brief  Sets the Xtal configuration in the ANA_FUNC_CONF0 register.
* @param  xXtal one of the possible value of the enum type XtalFrequency.
*         @arg XTAL_FLAG_24_MHz:  in case of 24 MHz crystal
*         @arg XTAL_FLAG_26_MHz:  in case of 26 MHz crystal
* @retval None.
*/
void SpiritRadioSetXtalFlag(XtalFlag xXtal)
{
  uint8_t tempRegValue = 0x00;
  
  /* Check the parameters */
  s_assert_param(IS_XTAL_FLAG(xXtal));
  
  /* Reads the ANA_FUNC_CONF_0 register */
  g_xStatus = SpiritSpiReadRegisters(ANA_FUNC_CONF0_BASE, 1, &tempRegValue);
  if(xXtal == XTAL_FLAG_26_MHz)
  {
    tempRegValue|=SELECT_24_26_MHZ_MASK;
  }
  else
  {
    tempRegValue &= (~SELECT_24_26_MHZ_MASK);
  }
  
  /* Sets the 24_26MHz_SELECT field in the ANA_FUNC_CONF_0 register */
  g_xStatus = SpiritSpiWriteRegisters(ANA_FUNC_CONF0_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the Xtal configuration in the ANA_FUNC_CONF0 register.
* @param  None.
* @retval XtalFrequency Settled Xtal configuration.
*/
XtalFlag SpiritRadioGetXtalFlag(void)
{
  uint8_t tempRegValue;
  
  /* Reads the Xtal configuration in the ANA_FUNC_CONF_0 register and return the value */
  g_xStatus = SpiritSpiReadRegisters(ANA_FUNC_CONF0_BASE, 1, &tempRegValue);
  
  return (XtalFlag)((tempRegValue & 0x40)>>6);
  
}


/**
* @brief  Returns the charge pump word for a given VCO frequency.
* @param  lFc channel center frequency expressed in Hz.
*         This parameter can be a value in one of the following ranges:<ul>
*         <li> High_Band: from 779 MHz to 915 MHz </li>
*         <li> Middle Band: from 387 MHz to 470 MHz </li>
*         <li> Low Band: from 300 MHz to 348 MHz </li>
*         <li> Very low Band: from 150 MHz to 174 MHz </li> </ul>
* @retval uint8_t Charge pump word.
*/
uint8_t SpiritRadioSearchWCP(uint32_t lFc)
{
  volatile SpiritBool find = S_FALSE;
  volatile SpiritBool find_up = S_FALSE;
  int8_t i=8;
  int8_t start = 0;
  int8_t end = 15;
  uint32_t vcofreq;
  uint8_t BFactor;
  
  /* Check the channel center frequency is in one of the possible range */
  s_assert_param(IS_FREQUENCY_BAND(lFc));
  
  /* Search the operating band */
  if(IS_FREQUENCY_BAND_HIGH(lFc))
  {
    BFactor = HIGH_BAND_FACTOR;
  }
  else if(IS_FREQUENCY_BAND_MIDDLE(lFc))
  {
    BFactor = MIDDLE_BAND_FACTOR;
  }
  else if(IS_FREQUENCY_BAND_LOW(lFc))
  {
    BFactor = LOW_BAND_FACTOR;
  }
  else if(IS_FREQUENCY_BAND_VERY_LOW(lFc))
  {
    BFactor = VERY_LOW_BAND_FACTOR;
  }
  
  /* Calculates the VCO frequency VCOFreq = lFc*B */
  vcofreq = (lFc/1000000)*BFactor;
  
  /* Search in the vco frequency array the charge pump word */
  if(vcofreq>=s_vectnVCOFreq[15])
  {
    i=15;
    find =S_TRUE;
  }
  while(!find)
  {
    if(vcofreq>=s_vectnVCOFreq[i])
    {
      if(vcofreq<=s_vectnVCOFreq[i+1])
      {
        find = S_TRUE;
        find_up = S_TRUE;
      }
      else
      {
        start = i;
        i = i+ (end - start)/2;
      }
    }
    else
    {
      if(vcofreq>=s_vectnVCOFreq[i-1])
        find = S_TRUE;
      else
      {
        end = i;
        i = i - (end-start)/2;
        if(end==i) {
          i=0;
        }        
      }
    }
  }
  if(find_up && i<15)
  {
    if((vcofreq - s_vectnVCOFreq[i])>=(s_vectnVCOFreq[i+1] - vcofreq))
    {
      i = i+1;
    }
  }
  else if(!find_up && i<15)
  {    
    if((s_vectnVCOFreq[i]-vcofreq)>=(vcofreq - s_vectnVCOFreq[i-1]))
    {
      i=i-1;
    }
  }  
  return  (uint8_t)(i%8);
  
}

/**
* @brief  Returns the synth word.
* @param  None.
* @retval uint32_t Synth word.
*/
uint32_t SpiritRadioGetSynthWord(void)
{
  uint8_t regArray[4];
  
  /* Reads the SYNTH registers, build the synth word and return it */
  g_xStatus = SpiritSpiReadRegisters(SYNT3_BASE, 4, regArray);
  return ((((uint32_t)(regArray[0]&0x1F))<<21)+(((uint32_t)(regArray[1]))<<13)+\
    (((uint32_t)(regArray[2]))<<5)+(((uint32_t)(regArray[3]))>>3));
  
}


/**
* @brief  Sets the SYNTH registers.
* @param  lSynthWord the synth word to write in the SYNTH[3:0] registers.
* @retval None.
*/
void SpiritRadioSetSynthWord(uint32_t lSynthWord)
{
  uint8_t tempArray[4];
  uint8_t tempRegValue;
  
  /* Reads the SYNT0 register */
  g_xStatus = SpiritSpiReadRegisters(SYNT0_BASE, 1, &tempRegValue);
  
  /* Mask the Band selected field */
  tempRegValue &= 0x07;
  
  /* Build the array for SYNTH registers */
  tempArray[0] = (uint8_t)((lSynthWord>>21)&(0x0000001F));
  tempArray[1] = (uint8_t)((lSynthWord>>13)&(0x000000FF));
  tempArray[2] = (uint8_t)((lSynthWord>>5)&(0x000000FF));
  tempArray[3] = (uint8_t)(((lSynthWord&0x0000001F)<<3)| tempRegValue);
  
  /* Writes the synth word in the SYNTH registers */
  g_xStatus = SpiritSpiWriteRegisters(SYNT3_BASE, 4, tempArray);
  
}


/**
* @brief  Sets the operating band.
* @param  xBand the band to set.
*         This parameter can be one of following parameters:
*         @arg  HIGH_BAND   High_Band selected: from 779 MHz to 915 MHz
*         @arg  MIDDLE_BAND: Middle Band selected: from 387 MHz to 470 MHz
*         @arg  LOW_BAND:  Low Band selected: from 300 MHz to 348 MHz
*         @arg  VERY_LOW_BAND:  Very low Band selected: from 150 MHz to 174 MHz
* @retval None.
*/
void SpiritRadioSetBand(BandSelect xBand)
{
  uint8_t tempRegValue;
  
  /* Check the parameters */
  s_assert_param(IS_BAND_SELECTED(xBand));
  
  /* Reads the SYNT0 register*/
  g_xStatus = SpiritSpiReadRegisters(SYNT0_BASE, 1, &tempRegValue);
  
  /* Mask the SYNTH[4;0] field and write the BS value */
  tempRegValue &= 0xF8;
  tempRegValue |= s_vectcBandRegValue[xBand];
  
  /* Configures the SYNT0 register setting the operating band */
  g_xStatus = SpiritSpiWriteRegisters(SYNT0_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the operating band.
* @param  None.
* @retval BandSelect Settled band.
*         This returned value can be one of the following parameters:
*         @arg  HIGH_BAND   High_Band selected: from 779 MHz to 915 MHz
*         @arg  MIDDLE_BAND: Middle Band selected: from 387 MHz to 470 MHz
*         @arg  LOW_BAND:  Low Band selected: from 300 MHz to 348 MHz
*         @arg  VERY_LOW_BAND:  Very low Band selected: from 150 MHz to 174 MHz
*/
BandSelect SpiritRadioGetBand(void)
{
  uint8_t tempRegValue;
  
  /* Reads the SYNT0 register */
  g_xStatus = SpiritSpiReadRegisters(SYNT0_BASE, 1, &tempRegValue);
  
  /* Mask the Band selected field */
  if((tempRegValue & 0x07) == SYNT0_BS_6)
  {
    return HIGH_BAND;
  }
  else if ((tempRegValue & 0x07) == SYNT0_BS_12)
  {
    return MIDDLE_BAND;
  }
  else if ((tempRegValue & 0x07) == SYNT0_BS_16)
  {
    return LOW_BAND;
  }
  else
  {
    return VERY_LOW_BAND;
  }
  
}


/**
* @brief  Sets the channel number.
* @param  cChannel the channel number.
* @retval None.
*/
void SpiritRadioSetChannel(uint8_t cChannel)
{
  /* Writes the CHNUM register */
  g_xStatus = SpiritSpiWriteRegisters(CHNUM_BASE, 1, &cChannel);
  
}


/**
* @brief  Returns the actual channel number.
* @param  None.
* @retval uint8_t Actual channel number.
*/
uint8_t SpiritRadioGetChannel(void)
{
  uint8_t tempRegValue;
  
  /* Reads the CHNUM register and return the value */
  g_xStatus = SpiritSpiReadRegisters(CHNUM_BASE, 1, &tempRegValue);
  
  return tempRegValue;
  
}


/**
* @brief  Sets the channel space factor in channel space register.
*         The channel spacing step is computed as F_Xo/32768.
* @param  fChannelSpace the channel space expressed in Hz.
* @retval None.
*/
void SpiritRadioSetChannelSpace(uint32_t fChannelSpace)
{
  uint8_t cChannelSpaceFactor;
  
  /* Round to the nearest integer */
  cChannelSpaceFactor = ((uint32_t)fChannelSpace*CHSPACE_DIVIDER)/s_lXtalFrequency;
  
  /* Write value into the register */
  g_xStatus = SpiritSpiWriteRegisters(CHSPACE_BASE, 1, &cChannelSpaceFactor);
  
}


/**
* @brief  Returns the channel space register.
* @param  None.
* @retval uint32_t Channel space. The channel space is: CS = channel_space_factor x XtalFrequency/2^15
*         where channel_space_factor is the CHSPACE register value.
*/
uint32_t SpiritRadioGetChannelSpace(void)
{
  uint8_t channelSpaceFactor;
  
  /* Reads the CHSPACE register, calculate the channel space and return it */
  g_xStatus = SpiritSpiReadRegisters(CHSPACE_BASE, 1, &channelSpaceFactor);
  
  /* Compute the Hertz value and return it */
  return ((channelSpaceFactor*s_lXtalFrequency)/CHSPACE_DIVIDER);
  
}


/**
* @brief  Sets the FC OFFSET register starting from xtal ppm value.
* @param  nXtalPpm the xtal offset expressed in ppm.
* @retval None.
*/
void SpiritRadioSetFrequencyOffsetPpm(int16_t nXtalPpm)
{
  uint8_t tempArray[2];
  int16_t xtalOffsetFactor;
  uint32_t synthWord, fBase;
  int32_t FOffsetTmp;
  BandSelect band;
  
  /* Reads the synth word */
  synthWord = SpiritRadioGetSynthWord();
  
  /* Reads the operating band */
  band = SpiritRadioGetBand();
  
  /* Calculates the frequency base */
  uint8_t cRefDiv = (uint8_t)SpiritRadioGetRefDiv()+1;
  fBase = synthWord*(s_lXtalFrequency/(s_vectcBHalfFactor[band]*cRefDiv)/FBASE_DIVIDER);
  
  /* Calculates the offset respect to RF frequency and according to xtal_ppm parameter */
  FOffsetTmp = (int32_t)((nXtalPpm*fBase)/PPM_FACTOR);
  
  /* Check the Offset is in the correct range */
  s_assert_param(IS_FREQUENCY_OFFSET(FOffsetTmp,s_lXtalFrequency));
  
  /* Calculates the FC_OFFSET value to write in the corresponding register */  
  xtalOffsetFactor = (FOffsetTmp*FBASE_DIVIDER)/s_lXtalFrequency;
  
  /* Build the array related to the FC_OFFSET_1 and FC_OFFSET_0 register */
  tempArray[0]=(uint8_t)((((uint16_t)xtalOffsetFactor)>>8)&0x0F);
  tempArray[1]=(uint8_t)(xtalOffsetFactor);
  
  /* Writes the FC_OFFSET registers */
  g_xStatus = SpiritSpiWriteRegisters(FC_OFFSET1_BASE, 2, tempArray);
  
}


/**
* @brief  Sets the FC OFFSET register starting from frequency offset expressed in Hz.
* @param  lFOffset frequency offset expressed in Hz as signed word.
* @retval None.
*/
void SpiritRadioSetFrequencyOffset(int32_t lFOffset)
{
  uint8_t tempArray[2];
  int16_t offset;
  
  /* Check that the Offset is in the correct range */
  s_assert_param(IS_FREQUENCY_OFFSET(lFOffset,s_lXtalFrequency));
  
  /* Calculates the offset value to write in the FC_OFFSET register */
  offset = (lFOffset*FBASE_DIVIDER)/s_lXtalFrequency;
  
  /* Build the array related to the FC_OFFSET_1 and FC_OFFSET_0 register */
  tempArray[0]=(uint8_t)((((uint16_t)offset)>>8)&0x0F);
  tempArray[1]=(uint8_t)(offset);
  
  /* Writes the FC_OFFSET registers */
  g_xStatus = SpiritSpiWriteRegisters(FC_OFFSET1_BASE, 2, tempArray);
  
}


/**
* @brief  Returns the actual frequency offset.
* @param  None.
* @retval int32_t Frequency offset expressed in Hz as signed word.
*/
int32_t SpiritRadioGetFrequencyOffset(void)
{
  uint8_t tempArray[2];
  int16_t xtalOffsetFactor;
  
  /* Reads the FC_OFFSET registers */
  g_xStatus = SpiritSpiReadRegisters(FC_OFFSET1_BASE, 2, tempArray);
  
  /* Calculates the Offset Factor */
  uint16_t xtalOffTemp = ((((uint16_t)tempArray[0])<<8)+((uint16_t)tempArray[1]));
  
  if(xtalOffTemp & 0x0800)
  {
    xtalOffTemp = xtalOffTemp | 0xF000;
  }
  else
  {
    xtalOffTemp = xtalOffTemp & 0x0FFF;
  }
  
  xtalOffsetFactor = *((int16_t*)(&xtalOffTemp));
  
  /* Calculates the frequency offset and return it */
  return ((int32_t)(xtalOffsetFactor*s_lXtalFrequency)/FBASE_DIVIDER);
  
}



/**
* @brief  Sets the Synth word and the Band Select register according to desired base carrier frequency.
*         In this API the Xtal configuration is read out from
*         the corresponding register. The user shall fix it before call this API.
* @param  lFBase the base carrier frequency expressed in Hz as unsigned word.
* @retval None.
*/
void SpiritRadioSetFrequencyBase(uint32_t lFBase)
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
  
  int32_t FOffset  = SpiritRadioGetFrequencyOffset();
  uint32_t lChannelSpace  = SpiritRadioGetChannelSpace();
  uint8_t cChannelNum = SpiritRadioGetChannel();
  
  /* Calculates the channel center frequency */
  Fc = lFBase + FOffset + lChannelSpace*cChannelNum;
  
  /* Reads the reference divider */
  uint8_t cRefDiv = (uint8_t)SpiritRadioGetRefDiv()+1;
  
  if(SpiritGeneralGetSpiritVersion() != SPIRIT_VERSION_2_0) 
  {
    /* Selects the VCO */
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
  
  synthWord = (uint32_t)(lFBase*s_vectcBHalfFactor[band]*(((double)(FBASE_DIVIDER*cRefDiv))/s_lXtalFrequency));
  
  /* Build the array of registers values for the analog part */
  anaRadioRegArray[0] = (uint8_t)(((synthWord>>21)&(0x0000001F))|(wcp<<5));
  anaRadioRegArray[1] = (uint8_t)((synthWord>>13)&(0x000000FF));
  anaRadioRegArray[2] = (uint8_t)((synthWord>>5)&(0x000000FF));
  anaRadioRegArray[3] = (uint8_t)(((synthWord&0x0000001F)<<3)| s_vectcBandRegValue[band]);
  
  /* Configures the needed Analog Radio registers */
  g_xStatus = SpiritSpiWriteRegisters(SYNT3_BASE, 4, anaRadioRegArray);
  
  SpiritManagementWaVcoCalibration();
}


/**
* @brief  Returns the base carrier frequency.
* @param  None.
* @retval uint32_t Base carrier frequency expressed in Hz as unsigned word.
*/
uint32_t SpiritRadioGetFrequencyBase(void)
{
  uint32_t synthWord;
  BandSelect band;
  
  /* Reads the synth word */
  synthWord = SpiritRadioGetSynthWord();
  
  /* Reads the operating band */
  band = SpiritRadioGetBand();
  
  uint8_t cRefDiv = (uint8_t)SpiritRadioGetRefDiv() + 1;
  
  /* Calculates the frequency base and return it */
  return (uint32_t)round(synthWord*(((double)s_lXtalFrequency)/(FBASE_DIVIDER*cRefDiv*s_vectcBHalfFactor[band])));
}


/**
* @brief  Returns the actual channel center frequency.
* @param  None.
* @retval uint32_t Actual channel center frequency expressed in Hz.
*/
uint32_t SpiritRadioGetCenterFrequency(void)
{
  int32_t offset;
  uint8_t channel;
  uint32_t fBase;
  uint32_t channelSpace;
  
  /* Reads the frequency base */
  fBase = SpiritRadioGetFrequencyBase();
  
  /* Reads the frequency offset */
  offset = SpiritRadioGetFrequencyOffset();
  
  /* Reads the channel space */
  channelSpace = SpiritRadioGetChannelSpace();
  
  /* Reads the channel number */
  channel = SpiritRadioGetChannel();
  
  /* Calculates the channel center frequency and return it */
  return (uint32_t)(fBase +  offset + (uint32_t)(channelSpace*channel));
  
}


/**
* @brief  Returns the mantissa and exponent, whose value used in the datarate formula
*         will give the datarate value closer to the given datarate.
* @param  fDatarate datarate expressed in bps. This parameter ranging between 100 and 500000.
* @param  pcM pointer to the returned mantissa value.
* @param  pcE pointer to the returned exponent value.
* @retval None.
*/
void SpiritRadioSearchDatarateME(uint32_t lDatarate, uint8_t* pcM, uint8_t* pcE)
{
  volatile SpiritBool find = S_FALSE;
  int8_t i=15;
  uint8_t cMantissaTmp;
  uint8_t cDivider = 0;
  
  /* Check the parameters */
  s_assert_param(IS_DATARATE(lDatarate));
  
  if(SpiritGeneralGetSpiritVersion() != SPIRIT_VERSION_2_0) 
  {
    cDivider = (uint8_t)SpiritRadioGetDigDiv();
  }
  
  /* Search in the datarate array the exponent value */
  while(!find && i>=0)
  {
    if(lDatarate>=(s_lXtalFrequency>>(20-i+cDivider)))
    {
      find = S_TRUE;
    }
    else
    {
      i--;
    }
  }
  i<0 ? i=0 : i;
  *pcE = i;
  
  /* Calculates the mantissa value according to the datarate formula */
  cMantissaTmp = (lDatarate*(1<<(23-i)))/(s_lXtalFrequency>>(5+cDivider))-256;
  
  /* Finds the mantissa value with less approximation */
  int16_t mantissaCalculation[3];
  for(uint8_t j=0;j<3;j++)
  {
    if((cMantissaTmp+j-1))
    {
      mantissaCalculation[j]=lDatarate-(((256+cMantissaTmp+j-1)*(s_lXtalFrequency>>(5+cDivider)))>>(23-i));
    }
    else
    {
      mantissaCalculation[j]=0x7FFF;
    }
  }
  uint16_t mantissaCalculationDelta = 0xFFFF; // TODO qui si puo' ottimizzare un attimo il codice
  for(uint8_t j=0;j<3;j++)
  {
    if(S_ABS(mantissaCalculation[j])<mantissaCalculationDelta)
    {
      mantissaCalculationDelta = S_ABS(mantissaCalculation[j]);
      *pcM = cMantissaTmp+j-1;
    }
  }
  
}


/**
* @brief  Returns the mantissa and exponent for a given bandwidth.
*         Even if it is possible to pass as parameter any value in the below mentioned range,
*         the API will search the closer value according to a fixed table of channel
*         bandwidth values (@ref s_vectnBandwidth), as defined in the datasheet, returning the corresponding mantissa
*         and exponent value.
* @param  lBandwidth bandwidth expressed in Hz. This parameter ranging between 1100 and 800100.
* @param  pcM pointer to the returned mantissa value.
* @param  pcE pointer to the returned exponent value.
* @retval None.
*/
void SpiritRadioSearchChannelBwME(uint32_t lBandwidth, uint8_t* pcM, uint8_t* pcE)
{
  volatile SpiritBool find = S_FALSE;
  volatile SpiritBool findUp = S_FALSE;
  int8_t i=44, i_tmp;
  int8_t start = 0;
  int8_t end = 89;
  uint8_t cDivider = 1;
  
  if(SpiritGeneralGetSpiritVersion() != SPIRIT_VERSION_2_0) 
  {
    /* Search in the channel filter bandwidth table the exponent value */
    if(SpiritRadioGetDigDiv())
    {
      cDivider = 2;
    }
    else
    {
      cDivider = 1;
    }
  }
  
  s_assert_param(IS_CH_BW(lBandwidth,s_lXtalFrequency/cDivider));
  
  uint32_t lChfltFactor = (s_lXtalFrequency/cDivider)/100;
  
  while(!find)
  {
    if(lBandwidth>(uint32_t)((s_vectnBandwidth26M[i]*lChfltFactor)/2600))
    {
      if(lBandwidth<=(uint32_t)((s_vectnBandwidth26M[i-1]*lChfltFactor)/2600))
      {
        find = S_TRUE;
        findUp = S_TRUE;
      }
      else
      {
        end = i;
        i = i - (end-start)/2;        
      }
    }
    else
    {
      if(lBandwidth>=(uint32_t)((s_vectnBandwidth26M[i+1]*lChfltFactor)/2600))
      {
        find = S_TRUE;
      }
      else
      {
        start = i;
        i = i+ (end - start)/2;
      }
    }
  }
  
  /* Finds the mantissa value with less approximation */
  i_tmp=i;
  int16_t chfltCalculation[3];
  for(uint8_t j=0;j<3;j++) 
  {
    if(((i_tmp+j-1)>=0) || ((i_tmp+j-1)<=89))
    {
      chfltCalculation[j] = lBandwidth - (uint32_t)((s_vectnBandwidth26M[i_tmp+j-1]*lChfltFactor)/2600);
    }
    else
    {
      chfltCalculation[j] = 0x7FFF;
    }
  }
  uint16_t chfltDelta = 0xFFFF;
  
  for(uint8_t j=0;j<3;j++)
  {
    if(S_ABS(chfltCalculation[j])<chfltDelta)
    {
      chfltDelta = S_ABS(chfltCalculation[j]);
      i=i_tmp+j-1;
    }    
  }
  
  *pcE = (uint8_t)(i/9);
  *pcM = (uint8_t)(i%9);
  
}


/**
* @brief  Returns the mantissa and exponent, whose value used in the frequency deviation formula
*         will give a frequency deviation value most closer to the given frequency deviation.
* @param  fFDev frequency deviation expressed in Hz. This parameter can be a value in the range [F_Xo*8/2^18, F_Xo*7680/2^18].
* @param  pcM pointer to the returned mantissa value.
* @param  pcE pointer to the returned exponent value.
* @retval None.
*/
void SpiritRadioSearchFreqDevME(uint32_t lFDev, uint8_t* pcM, uint8_t* pcE)
{
  volatile SpiritBool find = S_FALSE;
  int8_t i=9;
  uint8_t cMantissaTmp;
  uint8_t cAdder = 0;
  
  /* Check the parameters */
  s_assert_param(IS_F_DEV(lFDev,s_lXtalFrequency));
  
  if(SpiritGeneralGetSpiritVersion() != SPIRIT_VERSION_2_0) 
  {
    cAdder++;
  }
  
  /* Search in the frequency deviation array the exponent value */
  while(!find && i>=0)
  {
    if((uint32_t)lFDev>=(s_lXtalFrequency>>(cAdder+15-i)))
    {
      find = S_TRUE;
    }
    else
    {
      i--;
    }
  }
  i<0 ? i=0 : i;
  *pcE = i;
  
  cMantissaTmp = ((lFDev<<(12-i+cAdder))/(s_lXtalFrequency>>6)); 
  if(cMantissaTmp>7) {
    cMantissaTmp -= 8;
  }
  
  /* Finds the mantissa value with less approximation */
  int16_t mantissaCalculation[3];
  for(uint8_t j=0;j<3;j++) 
  {
    if((cMantissaTmp+j-1)<8)
    {
      mantissaCalculation[j] = lFDev - (((s_lXtalFrequency>>6)*(8+cMantissaTmp+j-1))>>(12-i+cAdder));
    }
    else
    {
      mantissaCalculation[j] = 0x7FFF;
    }
  }
  uint16_t mantissaDelta = 0xFFFF;
  
  for(uint8_t j=0;j<3;j++)
  {
    if(S_ABS(mantissaCalculation[j])<mantissaDelta)
    {
      mantissaDelta = S_ABS(mantissaCalculation[j]);
      *pcM=cMantissaTmp+j-1;
    }    
  }
}


/**
* @brief  Sets the datarate.
* @param  fDatarate datarate expressed in bps. This value shall be in the range
*         [100 500000].
* @retval None.
*/
void SpiritRadioSetDatarate(uint32_t lDatarate)
{
  uint8_t drE, tempRegValue[2];
  
  /* Check the parameters */
  s_assert_param(IS_DATARATE(lDatarate));
  
  /* Calculates the datarate mantissa and exponent */
  SpiritRadioSearchDatarateME(lDatarate, &tempRegValue[0], &drE);
  
  /* Reads the MOD_O register*/
  SpiritSpiReadRegisters(MOD0_BASE, 1, &tempRegValue[1]);
  
  /* Mask the other fields and set the datarate exponent */
  tempRegValue[1] &= 0xF0;
  tempRegValue[1] |= drE;
  
  /* Writes the Datarate registers */
  g_xStatus = SpiritSpiWriteRegisters(MOD1_BASE, 2, tempRegValue);
  
}


/**
* @brief  Returns the datarate.
* @param  None.
* @retval uint32_t Settled datarate expressed in bps.
*/
uint32_t SpiritRadioGetDatarate(void)
{
  uint8_t tempRegValue[2];
  uint8_t cDivider=0;
  
  /* Reads the datarate registers for mantissa and exponent */
  g_xStatus = SpiritSpiReadRegisters(MOD1_BASE, 2, tempRegValue);
  
  if(SpiritGeneralGetSpiritVersion() != SPIRIT_VERSION_2_0) 
  {
    /* Calculates the datarate */
    cDivider = 1-SpiritRadioGetDigDiv(); 
  }  
  return (((s_lXtalFrequency>>(5+cDivider))*(256+tempRegValue[0]))>>(23-(tempRegValue[1]&0x0F)));
  
}


/**
* @brief  Sets the frequency deviation.
* @param  fFDev frequency deviation expressed in Hz. Be sure that this value
*         is in the correct range [F_Xo*8/2^18, F_Xo*7680/2^18] Hz.
* @retval None.
*/
void SpiritRadioSetFrequencyDev(uint32_t lFDev)
{
  uint8_t FDevM, FDevE, tempRegValue;
  
  /* Check the parameters */
  s_assert_param(IS_F_DEV(lFDev, s_lXtalFrequency));
  
  /* Calculates the frequency deviation mantissa and exponent */
  SpiritRadioSearchFreqDevME(lFDev, &FDevM, &FDevE);
  
  /* Reads the FDEV0 register */
  SpiritSpiReadRegisters(FDEV0_BASE, 1, &tempRegValue);
  
  /* Mask the other fields and set the frequency deviation mantissa and exponent */
  tempRegValue &= 0x08;
  tempRegValue |= ((FDevE<<4)|(FDevM));
  
  /* Writes the Frequency deviation register */
  g_xStatus = SpiritSpiWriteRegisters(FDEV0_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the frequency deviation.
* @param  None.
* @retval uint32_t Frequency deviation value expressed in Hz.
*         This value will be in the range [F_Xo*8/2^18, F_Xo*7680/2^18] Hz.
*/
uint32_t SpiritRadioGetFrequencyDev(void)
{
  uint8_t tempRegValue, FDevM, FDevE;  
  uint8_t cAdder = 0;
  
  /* Reads the frequency deviation register for mantissa and exponent */
  g_xStatus = SpiritSpiReadRegisters(FDEV0_BASE, 1, &tempRegValue);
  FDevM = tempRegValue&0x07;
  FDevE = (tempRegValue&0xF0)>>4;
  
  if(SpiritGeneralGetSpiritVersion() != SPIRIT_VERSION_2_0) 
  {
    cAdder++;
  }
  
  /* Calculates the frequency deviation and return it */
  return (((s_lXtalFrequency>>6)*(8+FDevM))>>(12-FDevE+cAdder));
  
}


/**
* @brief  Sets the channel filter bandwidth.
* @param  lBandwidth channel filter bandwidth expressed in Hz. This parameter shall be in the range [1100 800100]
*         Even if it is possible to pass as parameter any value in the above mentioned range,
*         the API will search the most closer value according to a fixed table of channel
*         bandwidth values (@ref s_vectnBandwidth), as defined in the datasheet. To verify the settled channel bandwidth
*         it is possible to use the SpiritRadioGetChannelBW() API.
* @retval None.
*/
void SpiritRadioSetChannelBW(uint32_t lBandwidth)
{
  uint8_t bwM, bwE, tempRegValue;

  if(SpiritGeneralGetSpiritVersion() != SPIRIT_VERSION_2_0) 
  {
    /* Search in the channel filter bandwidth table the exponent value */
    if(SpiritRadioGetDigDiv())
    {
      s_assert_param(IS_CH_BW(lBandwidth,(s_lXtalFrequency/2)));
    }
    else
    {
      s_assert_param(IS_CH_BW(lBandwidth,(s_lXtalFrequency)));
    } 
  }

  /* Calculates the channel bandwidth mantissa and exponent */
  SpiritRadioSearchChannelBwME(lBandwidth, &bwM, &bwE);
  tempRegValue = (bwM<<4)|(bwE);
  
  /* Writes the Channel filter register */
  g_xStatus = SpiritSpiWriteRegisters(CHFLT_BASE, 1, &tempRegValue);
  
}

/**
* @brief  Returns the channel filter bandwidth.
* @param  None.
* @retval uint32_t Channel filter bandwidth expressed in Hz.
*/
uint32_t SpiritRadioGetChannelBW(void)
{
  uint8_t tempRegValue, bwM, bwE;
  
  /* Reads the channel filter register for mantissa and exponent */
  g_xStatus = SpiritSpiReadRegisters(CHFLT_BASE, 1, &tempRegValue);
  bwM = (tempRegValue&0xF0)>>4;
  bwE = tempRegValue&0x0F;
  
  /* Reads the channel filter bandwidth from the look-up table and return it */
  return (uint32_t)(100.0*s_vectnBandwidth26M[bwM+(bwE*9)]*s_lXtalFrequency/26e6);
  
}


/**
* @brief  Sets the modulation type.
* @param  xModulation modulation to set.
*         This parameter shall be of type @ref ModulationSelect .
* @retval None.
*/
void SpiritRadioSetModulation(ModulationSelect xModulation)
{
  uint8_t tempRegValue;
  
  /* Check the parameters */
  s_assert_param(IS_MODULATION_SELECTED(xModulation));
  
  /* Reads the modulation register */
  SpiritSpiReadRegisters(MOD0_BASE, 1, &tempRegValue);
  
  /* Mask the other fields and set the modulation type */
  tempRegValue &=0x8F;
  tempRegValue |= xModulation;
  
  /* Writes the modulation register */
  g_xStatus = SpiritSpiWriteRegisters(MOD0_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the modulation type used.
* @param  None.
* @retval ModulationSelect Settled modulation type.
*/
ModulationSelect SpiritRadioGetModulation(void)
{
  uint8_t tempRegValue;
  
  /* Reads the modulation register MOD0*/
  g_xStatus = SpiritSpiReadRegisters(MOD0_BASE, 1, &tempRegValue);
  
  /* Return the modulation type */
  return (ModulationSelect)(tempRegValue&0x70);
  
}


/**
* @brief  Enables or Disables the Continuous Wave transmit mode.
* @param  xNewState new state for power ramping.
*         This parameter can be: S_ENABLE or S_DISABLE .
* @retval None.
*/
void SpiritRadioCWTransmitMode(SpiritFunctionalState xNewState)
{
  uint8_t tempRegValue;
  
  /* Check the parameters */
  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(xNewState));
  
  /* Reads the modulation register MOD0 and mask the CW field */
  SpiritSpiReadRegisters(MOD0_BASE, 1, &tempRegValue);
  if(xNewState == S_ENABLE)
  {
    tempRegValue |=MOD0_CW;
  }
  else
  {
    tempRegValue &= (~MOD0_CW);
  }
  
  /* Writes the new value in the MOD0 register */
  g_xStatus = SpiritSpiWriteRegisters(MOD0_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Sets the OOK Peak Decay.
* @param  xOokDecay Peak decay control for OOK.
*         This parameter shall be of type @ref OokPeakDecay .
* @retval None.
*/
void SpiritRadioSetOokPeakDecay(OokPeakDecay xOokDecay)
{
  uint8_t tempRegValue;
  
  /* Check the parameters */
  s_assert_param(IS_OOK_PEAK_DECAY(xOokDecay));
  
  /* Reads the RSSI_FLT register */
  SpiritSpiReadRegisters(RSSI_FLT_BASE, 1, &tempRegValue);
  
  /* Mask the other fields and set OOK Peak Decay */
  tempRegValue &= 0xFC;
  tempRegValue |= xOokDecay;
  
  /* Writes the RSSI_FLT register to set the new OOK peak dacay value */
  g_xStatus = SpiritSpiWriteRegisters(RSSI_FLT_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the OOK Peak Decay.
* @param  None
* @retval OokPeakDecay Ook peak decay value.
*/
OokPeakDecay SpiritRadioGetOokPeakDecay(void)
{
  uint8_t tempRegValue;
  
  /* Reads the OOK peak decay register RSSI_FLT_BASE*/
  g_xStatus = SpiritSpiReadRegisters(RSSI_FLT_BASE, 1, &tempRegValue);
  
  /* Returns the OOK peak decay */
  return (OokPeakDecay) (tempRegValue & 0x03);
  
}


/**
* @brief  Configures the Power Amplifier Table and registers with value expressed in dBm.
* @param  cPALevelMaxIndex number of levels to set. This parameter shall be in the range [0:7].
* @param  cWidth step width expressed in terms of bit period units Tb/8.
*         This parameter shall be in the range [1:4].
* @param  xCLoad one of the possible value of the enum type PALoadCapacitor.
*         @arg LOAD_0_PF    No additional PA load capacitor
*         @arg LOAD_1_2_PF  1.2pF additional PA load capacitor
*         @arg LOAD_2_4_PF  2.4pF additional PA load capacitor
*         @arg LOAD_3_6_PF  3.6pF additional PA load capacitor
* @param  pfPAtabledBm pointer to an array of PA values in dbm between [-PA_LOWER_LIMIT: PA_UPPER_LIMIT] dbm.
*         The first element shall be the lower level (PA_LEVEL[0]) value and the last element
*         the higher level one (PA_LEVEL[paLevelMaxIndex]).
* @retval None.
*/
void SpiritRadioSetPATabledBm(uint8_t cPALevelMaxIndex, uint8_t cWidth, PALoadCapacitor xCLoad, float* pfPAtabledBm)
{
  uint8_t palevel[9], address, paLevelValue;
  float z;
  
  /* Check the parameters */
  s_assert_param(IS_PA_MAX_INDEX(cPALevelMaxIndex));
  s_assert_param(IS_PA_STEP_WIDTH(cWidth));
  s_assert_param(IS_PA_LOAD_CAP(xCLoad));
  
  /* Check the PA level in dBm is in the range and calculate the PA_LEVEL value
  to write in the corresponding register using the linearization formula */
  for(int i=0; i<=cPALevelMaxIndex; i++)
  {
    s_assert_param(IS_PAPOWER_DBM(*pfPAtabledBm));
    z=(*pfPAtabledBm+9.4089)/12.08;
    paLevelValue=(uint8_t)round(0.082812*z*z*z*z+1.0653*z*z*z-0.82942*z*z-28.146*z+46.242);
    paLevelValue>90 ? paLevelValue=90 : paLevelValue;
    palevel[cPALevelMaxIndex-i]=paLevelValue;
    pfPAtabledBm++;
  }
  
  /* Sets the PA_POWER[0] register */
  palevel[cPALevelMaxIndex+1]=xCLoad|(cWidth-1)<<3|cPALevelMaxIndex;
  
  /* Sets the base address */
  address=PA_POWER8_BASE+7-cPALevelMaxIndex;
  
  /* Configures the PA_POWER registers */
  g_xStatus = SpiritSpiWriteRegisters(address, cPALevelMaxIndex+2, palevel);
  
}


/**
* @brief  Returns the Power Amplifier Table and registers, returning values in dBm.
* @param  pcPALevelMaxIndex pointer to the number of levels settled.
*         This parameter will be in the range [0:7].
* @param  pfPAtabledBm pointer to an array of 8 elements containing the PA value in dbm.
*         The first element will be the PA_LEVEL_0 and the last element
*         will be PA_LEVEL_7. Any value higher than PA_UPPER_LIMIT implies no output
*         power (output stage is in high impedance).
* @retval None.
*/
void SpiritRadioGetPATabledBm(uint8_t* pcPALevelMaxIndex, float* pfPAtabledBm)
{
  uint8_t palevelvect[9];
  float z;
  
  /* Reads the PA_LEVEL_x registers and the PA_POWER_0 register */
  g_xStatus = SpiritSpiReadRegisters(PA_POWER8_BASE, 9, palevelvect);
  
  /* Fill the PAtable */
  for(int i=7; i>=0; i--)
  {
    z= (palevelvect[i]-45.456)/26.037;
    *pfPAtabledBm = 0.17524*z*z*z*z -0.63836*z*z*z -1.0048*z*z -10.236*z -8.7448;
    pfPAtabledBm++;
  }
  
  /* Return the settled index */
  *pcPALevelMaxIndex = palevelvect[8]&0x07;
  
}


/**
* @brief  Sets a specific PA_LEVEL register, with a value given in dBm.
* @param  cIndex PA_LEVEL to set. This parameter shall be in the range [0:7].
* @param  fPowerdBm PA value to write expressed in dBm . Be sure that this values is in the
*         correct range [-PA_LOWER_LIMIT: PA_UPPER_LIMIT] dBm.
* @retval None.
*/
void SpiritRadioSetPALeveldBm(uint8_t cIndex, float fPowerdBm)
{
  uint8_t address, paLevelValue;
  
  /* Check the parameters */
  s_assert_param(IS_PA_MAX_INDEX(cIndex));
  s_assert_param(IS_PAPOWER_DBM(fPowerdBm));
  
  /* Calculates the PA level value to write in the corresponding register using
  an interpolation formula */
  float z=(fPowerdBm+9.4089)/12.08;
  paLevelValue=(uint8_t)round(0.082812*z*z*z*z+1.0653*z*z*z-0.82942*z*z-28.146*z+46.242);
  
  paLevelValue>90 ? paLevelValue=90 : paLevelValue;
  
  /* Sets the base address */
  address=PA_POWER8_BASE+7-cIndex;
  
  /* Configures the PA_LEVEL register */
  g_xStatus = SpiritSpiWriteRegisters(address, 1, &paLevelValue);
  
}


/**
* @brief  Returns a specific PA_LEVEL register, returning a value in dBm.
* @param  cIndex PA_LEVEL to read. This parameter shall be in the range [0:7]
* @retval float Settled power level expressed in dBm. A value
*         higher than PA_UPPER_LIMIT dBm implies no output power
*         (output stage is in high impedance).
*/
float SpiritRadioGetPALeveldBm(uint8_t cIndex)
{
  uint8_t address, paLevelValue;
  
  /* Check the parameters */
  s_assert_param(IS_PA_MAX_INDEX(cIndex));
  
  /* Sets the base address */
  address=PA_POWER8_BASE+7-cIndex;
  
  /* Reads the PA_LEVEL[cIndex] register */
  g_xStatus = SpiritSpiReadRegisters(address, 1, &paLevelValue);
  
  //  float z= (paLevelValue-45.456)/26.037;
  float z=(paLevelValue-45.5)/26.125;
  
  /* Calculates the power level in dBm using the linearization formula and return the value */
  return -0.0045522*z*z*z*z-0.60856*z*z*z-0.3432*z*z-10.976*z-9.0615;
  
}


/**
* @brief  Configures the Power Amplifier Table and registers.
* @param  cPALevelMaxIndex number of levels to set. This parameter shall be in the range [0:7].
* @param  cWidth step width expressed in terms of bit period units Tb/8.
*         This parameter shall be in the range [1:4].
* @param  xCLoad one of the possible value of the enum type PALoadCapacitor.
*         @arg LOAD_0_PF    No additional PA load capacitor
*         @arg LOAD_1_2_PF  1.2pF additional PA load capacitor
*         @arg LOAD_2_4_PF  2.4pF additional PA load capacitor
*         @arg LOAD_3_6_PF  3.6pF additional PA load capacitor
* @param  pcPAtable pointer to an array of PA values in the range [0: 90], where 0 implies no
*         output power, 1 will be the maximum level and 90 the minimum one
*         The first element shall be the lower level (PA_LEVEL[0]) value and the last element
*         the higher level one (PA_LEVEL[paLevelMaxIndex]).
* @retval None.
*/
void SpiritRadioSetPATable(uint8_t cPALevelMaxIndex, uint8_t cWidth, PALoadCapacitor xCLoad, uint8_t* pcPAtable)
{
  uint8_t palevel[9], address;
  
  /* Check the parameters */
  s_assert_param(IS_PA_MAX_INDEX(cPALevelMaxIndex));
  s_assert_param(IS_PA_STEP_WIDTH(cWidth));
  s_assert_param(IS_PA_LOAD_CAP(xCLoad));
  
  /* Check the PA levels are in the range */
  for(int i=0; i<=cPALevelMaxIndex; i++)
  {
    s_assert_param(IS_PAPOWER(*pcPAtable));
    palevel[cPALevelMaxIndex-i]=*pcPAtable;
    pcPAtable++;
  }
  
  /* Sets the PA_POWER[0] register */
  palevel[cPALevelMaxIndex+1]=xCLoad|((cWidth-1)<<3)|cPALevelMaxIndex;
  
  /* Sets the base address */
  address=PA_POWER8_BASE+7-cPALevelMaxIndex;
  
  /* Configures the PA_POWER registers */
  g_xStatus = SpiritSpiWriteRegisters(address, cPALevelMaxIndex+2, palevel);
  
}


/**
* @brief  Returns the Power Amplifier Table and registers.
* @param  pcPALevelMaxIndex pointer to the number of levels settled.
*         This parameter shall be in the range [0:7].
* @param  pcPAtable pointer to an array of 8 elements containing the PA value.
*         The first element will be the PA_LEVEL_0 and the last element
*         will be PA_LEVEL_7. Any value equals to 0 implies that level has
*         no output power (output stage is in high impedance).
* @retval None
*/
void SpiritRadioGetPATable(uint8_t* pcPALevelMaxIndex, uint8_t* pcPAtable)
{
  uint8_t palevelvect[9];
  
  /* Reads the PA_LEVEL_x registers and the PA_POWER_0 register */
  g_xStatus = SpiritSpiReadRegisters(PA_POWER8_BASE, 9, palevelvect);
  
  /* Fill the PAtable */
  for(int i=7; i>=0; i--)
  {
    *pcPAtable = palevelvect[i];
    pcPAtable++;
  }
  
  /* Return the settled index */
  *pcPALevelMaxIndex = palevelvect[8]&0x07;
  
}


/**
* @brief  Sets a specific PA_LEVEL register.
* @param  cIndex PA_LEVEL to set. This parameter shall be in the range [0:7].
* @param  cPower PA value to write in the register. Be sure that this values is in the
*         correct range [0 : 90].
* @retval None.
*/
void SpiritRadioSetPALevel(uint8_t cIndex, uint8_t cPower)
{
  uint8_t address;
  
  /* Check the parameters */
  s_assert_param(IS_PA_MAX_INDEX(cIndex));
  s_assert_param(IS_PAPOWER(cPower));
  
  /* Sets the base address */
  address=PA_POWER8_BASE+7-cIndex;
  
  /* Configures the PA_LEVEL register */
  g_xStatus = SpiritSpiWriteRegisters(address, 1, &cPower);
  
}


/**
* @brief  Returns a specific PA_LEVEL register.
* @param  cIndex PA_LEVEL to read. This parameter shall be in the range [0:7].
* @retval uint8_t PA_LEVEL value. A value equal to zero
*         implies no output power (output stage is in high impedance).
*/
uint8_t SpiritRadioGetPALevel(uint8_t cIndex)
{
  uint8_t address, tempRegValue;
  
  /* Check the parameters */
  s_assert_param(IS_PA_MAX_INDEX(cIndex));
  
  /* Sets the base address */
  address=PA_POWER8_BASE+7-cIndex;
  
  /* Reads the PA_LEVEL[cIndex] register and return the value */
  g_xStatus = SpiritSpiReadRegisters(address, 1, &tempRegValue);
  return tempRegValue;
  
}


/**
* @brief  Sets the output stage additional load capacitor bank.
* @param  xCLoad one of the possible value of the enum type PALoadCapacitor.
*         @arg LOAD_0_PF    No additional PA load capacitor
*         @arg LOAD_1_2_PF  1.2pF additional PA load capacitor
*         @arg LOAD_2_4_PF  2.4pF additional PA load capacitor
*         @arg LOAD_3_6_PF  3.6pF additional PA load capacitor
* @retval None.
*/
void SpiritRadioSetPACwc(PALoadCapacitor xCLoad)
{
  uint8_t tempRegValue;
  
  /* Check the parameters */
  s_assert_param(IS_PA_LOAD_CAP(xCLoad));
  
  /* Reads the PA_POWER_0 register */
  SpiritSpiReadRegisters(PA_POWER0_BASE, 1, &tempRegValue);
  
  /* Mask the CWC[1:0] field and write the new value */
  tempRegValue &= 0x3F;
  tempRegValue |= xCLoad;
  
  /* Configures the PA_POWER_0 register */
  g_xStatus = SpiritSpiWriteRegisters(PA_POWER0_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the output stage additional load capacitor bank.
* @param  None.
* @retval PALoadCapacitor Output stage additional load capacitor bank.
*         This parameter can be:
*         @arg LOAD_0_PF    No additional PA load capacitor
*         @arg LOAD_1_2_PF  1.2pF additional PA load capacitor
*         @arg LOAD_2_4_PF  2.4pF additional PA load capacitor
*         @arg LOAD_3_6_PF  3.6pF additional PA load capacitor
*/
PALoadCapacitor SpiritRadioGetPACwc(void)
{
  uint8_t tempRegValue;
  
  /* Reads the PA_POWER_0 register */
  g_xStatus = SpiritSpiReadRegisters(PA_POWER0_BASE, 1, &tempRegValue);
  
  /* Mask the CWC[1:0] field and return the value*/
  return (PALoadCapacitor)(tempRegValue & 0xC0);
  
}


/**
* @brief  Sets a specific PA_LEVEL_MAX_INDEX.
* @param  cIndex PA_LEVEL_MAX_INDEX to set. This parameter shall be in the range [0:7].
* @retval None
*/
void SpiritRadioSetPALevelMaxIndex(uint8_t cIndex)
{
  uint8_t tempRegValue;
  
  /* Check the parameters */
  s_assert_param(IS_PA_MAX_INDEX(cIndex));
  
  /* Reads the PA_POWER_0 register */
  SpiritSpiReadRegisters(PA_POWER0_BASE, 1, &tempRegValue);
  
  /* Mask the PA_LEVEL_MAX_INDEX[1:0] field and write the new value */
  tempRegValue &= 0xF8;
  tempRegValue |= cIndex;
  
  /* Configures the PA_POWER_0 register */
  g_xStatus = SpiritSpiWriteRegisters(PA_POWER0_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the actual PA_LEVEL_MAX_INDEX.
* @param  None.
* @retval uint8_t Actual PA_LEVEL_MAX_INDEX. This parameter will be in the range [0:7].
*/
uint8_t SpiritRadioGetPALevelMaxIndex(void)
{
  uint8_t tempRegValue;
  
  /* Reads the PA_POWER_0 register */
  g_xStatus = SpiritSpiReadRegisters(PA_POWER0_BASE, 1, &tempRegValue);
  
  /* Mask the PA_LEVEL_MAX_INDEX[1:0] field and return the value */
  return (tempRegValue & 0x07);
  
}


/**
* @brief  Sets a specific PA_RAMP_STEP_WIDTH.
* @param  cWidth step width expressed in terms of bit period units Tb/8.
*         This parameter shall be in the range [1:4].
* @retval None.
*/
void SpiritRadioSetPAStepWidth(uint8_t cWidth)
{
  uint8_t tempRegValue;
  
  /* Check the parameters */
  s_assert_param(IS_PA_STEP_WIDTH(cWidth));
  
  /* Reads the PA_POWER_0 register */
  SpiritSpiReadRegisters(PA_POWER0_BASE, 1, &tempRegValue);
  
  /* Mask the PA_RAMP_STEP_WIDTH[1:0] field and write the new value */
  tempRegValue &= 0xE7;
  tempRegValue |= (cWidth-1)<<3;
  
  /* Configures the PA_POWER_0 register */
  g_xStatus = SpiritSpiWriteRegisters(PA_POWER0_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the actual PA_RAMP_STEP_WIDTH.
* @param  None.
* @retval uint8_t Step width value expressed in terms of bit period units Tb/8.
*         This parameter will be in the range [1:4].
*/
uint8_t SpiritRadioGetPAStepWidth(void)
{
  uint8_t tempRegValue;
  
  /* Reads the PA_POWER_0 register */
  g_xStatus = SpiritSpiReadRegisters(PA_POWER0_BASE, 1, &tempRegValue);
  
  /* Mask the PA_RAMP_STEP_WIDTH[1:0] field and return the value */
  tempRegValue &= 0x18;
  return  ((tempRegValue>>3)+1);
  
}


/**
* @brief  Enables or Disables the Power Ramping.
* @param  xNewState new state for power ramping.
*         This parameter can be: S_ENABLE or S_DISABLE.
* @retval None.
*/
void SpiritRadioPARamping(SpiritFunctionalState xNewState)
{
  uint8_t tempRegValue = 0x00;
  
  /* Check the parameters */
  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(xNewState));
  
  /* Reads the PA_POWER_0 register and configure the PA_RAMP_ENABLE field */
  SpiritSpiReadRegisters(PA_POWER0_BASE, 1, &tempRegValue);
  if(xNewState == S_ENABLE)
  {
    tempRegValue |= PA_POWER0_PA_RAMP_MASK;
  }
  else
  {
    tempRegValue &= (~PA_POWER0_PA_RAMP_MASK);
  }
  
  /* Sets the PA_POWER_0 register */
  g_xStatus = SpiritSpiWriteRegisters(PA_POWER0_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the Power Ramping enable bit.
* @param  xNewState new state for power ramping.
*         This parameter can be: S_ENABLE or S_DISABLE.
* @retval None.
*/
SpiritFunctionalState SpiritRadioGetPARamping(void)
{
  uint8_t tempRegValue;
  
  /* Reads the PA_POWER_0 register and configure the PA_RAMP_ENABLE field */
  g_xStatus = SpiritSpiReadRegisters(PA_POWER0_BASE, 1, &tempRegValue);
  
  /* Mask and return data */
  return (SpiritFunctionalState)((tempRegValue>>5) & 0x01);
  
}


/**
* @brief  Enables or Disables the AFC.
* @param  xNewState new state for AFC.
*         This parameter can be: S_ENABLE or S_DISABLE.
* @retval None.
*/
void SpiritRadioAFC(SpiritFunctionalState xNewState)
{
  uint8_t tempRegValue = 0x00;
  
  /* Check the parameters */
  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(xNewState));
  
  /* Reads the AFC_2 register and configure the AFC Enabled field */
  SpiritSpiReadRegisters(AFC2_BASE, 1, &tempRegValue);
  if(xNewState == S_ENABLE)
  {
    tempRegValue |= AFC2_AFC_MASK;
  }
  else
  {
    tempRegValue &= (~AFC2_AFC_MASK);
  }
  
  /* Sets the AFC_2 register */
  g_xStatus = SpiritSpiWriteRegisters(AFC2_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Enables or Disables the AFC freeze on sync word detection.
* @param  xNewState new state for AFC freeze on sync word detection.
*         This parameter can be: S_ENABLE or S_DISABLE.
* @retval None.
*/
void SpiritRadioAFCFreezeOnSync(SpiritFunctionalState xNewState)
{
  uint8_t tempRegValue = 0x00;
  
  /* Check the parameters */
  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(xNewState));
  
  /* Reads the AFC_2 register and configure the AFC Freeze on Sync field */
  SpiritSpiReadRegisters(AFC2_BASE, 1, &tempRegValue);
  if(xNewState == S_ENABLE)
  {
    tempRegValue |= AFC2_AFC_FREEZE_ON_SYNC_MASK;
  }
  else
  {
    tempRegValue &= (~AFC2_AFC_FREEZE_ON_SYNC_MASK);
  }
  
  /* Sets the AFC_2 register */
  g_xStatus = SpiritSpiWriteRegisters(AFC2_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Sets the AFC working mode.
* @param  xMode the AFC mode. This parameter can be one of the values defined in @ref AFCMode :
*         @arg AFC_SLICER_CORRECTION     AFC loop closed on slicer
*         @arg AFC_2ND_IF_CORRECTION     AFC loop closed on 2nd conversion stage
* @retval None.
*/
void SpiritRadioSetAFCMode(AFCMode xMode)
{
  uint8_t tempRegValue = 0x00;
  
  /* Check the parameters */
  s_assert_param(IS_AFC_MODE(xMode));
  
  /* Reads the AFC_2 register and configure the AFC Mode field */
  SpiritSpiReadRegisters(AFC2_BASE, 1, &tempRegValue);
  if(xMode == AFC_2ND_IF_CORRECTION)
  {
    tempRegValue |= AFC_2ND_IF_CORRECTION;
  }
  else
  {
    tempRegValue &= (~AFC_2ND_IF_CORRECTION);
  }
  
  /* Sets the AFC_2 register */
  g_xStatus = SpiritSpiWriteRegisters(AFC2_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the AFC working mode.
* @param  None.
* @retval AFCMode Settled AFC mode. This parameter will be one of the values defined in @ref AFCMode :
*         @arg AFC_SLICER_CORRECTION     AFC loop closed on slicer
*         @arg AFC_2ND_IF_CORRECTION     AFC loop closed on 2nd conversion stage
*/
AFCMode SpiritRadioGetAFCMode(void)
{
  uint8_t tempRegValue;
  
  /* Reads the AFC_2 register */
  g_xStatus = SpiritSpiReadRegisters(AFC2_BASE, 1, &tempRegValue);
  
  /* Mask the AFC Mode field and returns the value */
  return (AFCMode)(tempRegValue & 0x20);
  
}


/**
* @brief  Sets the AFC peak detector leakage.
* @param  cLeakage the peak detector leakage. This parameter shall be in the range:
*         [0:31].
* @retval None.
*/
void SpiritRadioSetAFCPDLeakage(uint8_t cLeakage)
{
  uint8_t tempRegValue = 0x00;
  
  /* Check the parameters */
  s_assert_param(IS_AFC_PD_LEAKAGE(cLeakage));
  
  /* Reads the AFC_2 register and configure the AFC PD leakage field */
  SpiritSpiReadRegisters(AFC2_BASE, 1, &tempRegValue);
  tempRegValue &= 0xE0;
  tempRegValue |= cLeakage;
  
  /* Sets the AFC_2 register */
  g_xStatus = SpiritSpiWriteRegisters(AFC2_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the AFC peak detector leakage.
* @param  None.
* @retval uint8_t Peak detector leakage value. This parameter will be in the range:
*         [0:31].
*/
uint8_t SpiritRadioGetAFCPDLeakage(void)
{
  uint8_t tempRegValue;
  
  /* Reads the AFC_2 register */
  g_xStatus = SpiritSpiReadRegisters(AFC2_BASE, 1, &tempRegValue);
  
  /* Mask the AFC PD leakage field and return the value */
  return (tempRegValue & 0x1F);
  
}


/**
* @brief  Sets the length of the AFC fast period expressed as number of samples.
* @param  cLength length of the fast period in number of samples.
* @retval None.
*/
void SpiritRadioSetAFCFastPeriod(uint8_t cLength)
{
  /* Sets the AFC_1 register */
  g_xStatus = SpiritSpiWriteRegisters(AFC1_BASE, 1, &cLength);
  
}


/**
* @brief  Returns the AFC fast period expressed as number of samples.
* @param  None.
* @retval uint8_t Length of the fast period in number of samples.
*/
uint8_t SpiritRadioGetAFCFastPeriod(void)
{
  uint8_t tempRegValue;
  
  /* Reads the AFC 1 register and return the value */
  g_xStatus = SpiritSpiReadRegisters(AFC1_BASE, 1, &tempRegValue);
  
  return tempRegValue;
  
}


/**
* @brief  Sets the AFC loop gain in fast mode.
* @param  cGain AFC loop gain in fast mode. This parameter shall be in the range:
*         [0:15].
* @retval None.
*/
void SpiritRadioSetAFCFastGain(uint8_t cGain)
{
  uint8_t tempRegValue = 0x00;
  
  /* Check the parameters */
  s_assert_param(IS_AFC_FAST_GAIN(cGain));
  
  /* Reads the AFC_0 register and configure the AFC Fast Gain field */
  SpiritSpiReadRegisters(AFC0_BASE, 1, &tempRegValue);
  tempRegValue &= 0x0F;
  tempRegValue |= cGain<<4;
  
  /* Sets the AFC_0 register */
  g_xStatus = SpiritSpiWriteRegisters(AFC0_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the AFC loop gain in fast mode.
* @param  None.
* @retval uint8_t AFC loop gain in fast mode. This parameter will be in the range:
*         [0:15].
*/
uint8_t SpiritRadioGetAFCFastGain(void)
{
  uint8_t tempRegValue;
  
  /* Reads the AFC_0 register, mask the AFC Fast Gain field and return the value  */
  g_xStatus = SpiritSpiReadRegisters(AFC0_BASE, 1, &tempRegValue);
  
  return ((tempRegValue & 0xF0)>>4);
  
}


/**
* @brief  Sets the AFC loop gain in slow mode.
* @param  cGain AFC loop gain in slow mode. This parameter shall be in the range:
*         [0:15].
* @retval None.
*/
void SpiritRadioSetAFCSlowGain(uint8_t cGain)
{
  uint8_t tempRegValue = 0x00;
  
  /* Check the parameters */
  s_assert_param(IS_AFC_SLOW_GAIN(cGain));
  
  /* Reads the AFC_0 register and configure the AFC Slow Gain field */
  SpiritSpiReadRegisters(AFC0_BASE, 1, &tempRegValue);
  tempRegValue &= 0xF0;
  tempRegValue |= cGain;
  
  /* Sets the AFC_0 register */
  g_xStatus = SpiritSpiWriteRegisters(AFC0_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the AFC loop gain in slow mode.
* @param  None.
* @retval uint8_t AFC loop gain in slow mode. This parameter will be in the range:
*         [0:15].
*/
uint8_t SpiritRadioGetAFCSlowGain(void)
{
  uint8_t tempRegValue;
  
  /* Reads the AFC_0 register, mask the AFC Slow Gain field and return the value */
  g_xStatus = SpiritSpiReadRegisters(AFC0_BASE, 1, &tempRegValue);
  
  return (tempRegValue & 0x0F);
  
}


/**
* @brief  Returns the AFC correction from the corresponding register.
* @param  None.
* @retval int8_t AFC correction, read from the corresponding register.
*         This parameter will be in the range [-128:127].
*/
int8_t SpiritRadioGetAFCCorrectionReg(void)
{
  uint8_t tempRegValue;
  
  /* Reads the AFC_CORR register, cast the read value as signed char and return it */
  g_xStatus = SpiritSpiReadRegisters(AFC_CORR_BASE, 1, &tempRegValue);
  
  return (int8_t)tempRegValue;
  
}


/**
* @brief  Returns the AFC correction expressed in Hz.
* @param  None.
* @retval int32_t AFC correction expressed in Hz
*         according to the following formula:<ul>
*         <li> Fafc[Hz]= (Fxtal/(12*2^10))*2*AFC_CORR  where </li>
*         <li> AFC_CORR is the value read in the AFC_CORR register </li> </ul>
*/
int32_t SpiritRadioGetAFCCorrectionHz(void)
{
  int8_t correction;
  
  /* Reads the AFC correction register */
  correction = SpiritRadioGetAFCCorrectionReg();
  
  /* Calculates and return the Frequency Correction */
  return (int32_t)(s_lXtalFrequency/(12*pow(2,10))*2*correction);
  
}


/**
* @brief  Corrects the frequency offset through the AFC mechanism.
* @param  None.
* @retval None.
*/
void SpiritRadioAutoSetFOffset(void)
{
  int32_t fOffset;
  
  /* Reads the AFC correction and the actual frequency offset */
  fOffset = SpiritRadioGetFrequencyOffset() + SpiritRadioGetAFCCorrectionHz();
  
  /* Re-Sets the new frequency offset */
  SpiritRadioSetFrequencyOffset(fOffset);
  
}


/**
* @brief  Enables or Disables the AGC.
* @param  xNewState new state for AGC.
*         This parameter can be: S_ENABLE or S_DISABLE
* @retval None.
*/
void SpiritRadioAGC(SpiritFunctionalState xNewState)
{
  uint8_t tempRegValue = 0x00;
  
  /* Check the parameters */
  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(xNewState));
  
  /* Reads the AGCCTRL_0 register and configure the AGC Enabled field */
  SpiritSpiReadRegisters(AGCCTRL0_BASE, 1, &tempRegValue);
  if(xNewState == S_ENABLE)
  {
    tempRegValue |= AGCCTRL0_AGC_MASK;
  }
  else
  {
    tempRegValue &= (~AGCCTRL0_AGC_MASK);
  }
  
  /* Sets the AGCCTRL_0 register */
  g_xStatus = SpiritSpiWriteRegisters(AGCCTRL0_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Sets the AGC working mode.
* @param  xMode the AGC mode. This parameter can be one of the values defined in @ref AGCMode :
*         @arg AGC_LINEAR_MODE     AGC works in linear mode
*         @arg AGC_BINARY_MODE     AGC works in binary mode
* @retval None.
*/
void SpiritRadioSetAGCMode(AGCMode xMode)
{
  uint8_t tempRegValue = 0x00;
  
  /* Check the parameters */
  s_assert_param(IS_AGC_MODE(xMode));
  
  /* Reads the AGCCTRL_0 register and configure the AGC Mode field */
  SpiritSpiReadRegisters(AGCCTRL0_BASE, 1, &tempRegValue);
  if(xMode == AGC_BINARY_MODE)
  {
    tempRegValue |= AGC_BINARY_MODE;
  }
  else
  {
    tempRegValue &= (~AGC_BINARY_MODE);
  }
  
  /* Sets the AGCCTRL_0 register */
  g_xStatus = SpiritSpiWriteRegisters(AGCCTRL0_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the AGC working mode.
* @param  None.
* @retval AGCMode Settled AGC mode.  This parameter can be one of the values defined in @ref AGCMode :
*         @arg AGC_LINEAR_MODE     AGC works in linear mode
*         @arg AGC_BINARY_MODE     AGC works in binary mode
*/
AGCMode SpiritRadioGetAGCMode(void)
{
  uint8_t tempRegValue;
  
  /* Reads the AGCCTRL_0 register, mask the AGC Mode field and return the value */
  g_xStatus = SpiritSpiReadRegisters(AGCCTRL0_BASE, 1, &tempRegValue);
  
  return  (AGCMode)(tempRegValue & 0x40);
  
}


/**
* @brief  Enables or Disables the AGC freeze on steady state.
* @param  xNewState new state for AGC freeze on steady state.
*         This parameter can be: S_ENABLE or S_DISABLE.
* @retval None.
*/
void SpiritRadioAGCFreezeOnSteady(SpiritFunctionalState xNewState)
{
  uint8_t tempRegValue = 0x00;
  
  /* Check the parameters */
  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(xNewState));
  
  /* Reads the AGCCTRL_2 register and configure the AGC Freeze On Steady field */
  SpiritSpiReadRegisters(AGCCTRL2_BASE, 1, &tempRegValue);
  if(xNewState == S_ENABLE)
  {
    tempRegValue |= AGCCTRL2_FREEZE_ON_STEADY_MASK;
  }
  else
  {
    tempRegValue &= (~AGCCTRL2_FREEZE_ON_STEADY_MASK);
  }
  
  /* Sets the AGCCTRL_2 register */
  g_xStatus = SpiritSpiWriteRegisters(AGCCTRL2_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Enable or Disable the AGC freeze on sync detection.
* @param  xNewState new state for AGC freeze on sync detection.
*         This parameter can be: S_ENABLE or S_DISABLE.
* @retval None.
*/
void SpiritRadioAGCFreezeOnSync(SpiritFunctionalState xNewState)
{
  uint8_t tempRegValue = 0x00;
  
  /* Check the parameters */
  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(xNewState));
  
  /* Reads the AGCCTRL_2 register and configure the AGC Freeze On Sync field */
  SpiritSpiReadRegisters(AGCCTRL2_BASE, 1, &tempRegValue);
  if(xNewState == S_ENABLE)
  {
    tempRegValue |= AGCCTRL2_FREEZE_ON_SYNC_MASK;
  }
  else
  {
    tempRegValue &= (~AGCCTRL2_FREEZE_ON_SYNC_MASK);
  }
  
  /* Sets the AGCCTRL_2 register */
  g_xStatus = SpiritSpiWriteRegisters(AGCCTRL2_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Enable or Disable the AGC to start with max attenuation.
* @param  xNewState new state for AGC start with max attenuation mode.
*         This parameter can be: S_ENABLE or S_DISABLE.
* @retval None.
*/
void SpiritRadioAGCStartMaxAttenuation(SpiritFunctionalState xNewState)
{
  uint8_t tempRegValue = 0x00;
  
  /* Check the parameters */
  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(xNewState));
  
  /* Reads the AGCCTRL_2 register and configure the AGC Start Max Attenuation field */
  SpiritSpiReadRegisters(AGCCTRL2_BASE, 1, &tempRegValue);
  if(xNewState == S_ENABLE)
  {
    tempRegValue |= AGCCTRL2_START_MAX_ATTENUATION_MASK;
  }
  else
  {
    tempRegValue &= (~AGCCTRL2_START_MAX_ATTENUATION_MASK);
  }
  
  /* Sets the AGCCTRL_2 register */
  g_xStatus = SpiritSpiWriteRegisters(AGCCTRL2_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Sets the AGC measure time.
* @param  nTime AGC measure time expressed in us. This parameter shall be in the range [0, 393216/F_Xo].
* @retval None.
*/
void SpiritRadioSetAGCMeasureTimeUs(uint16_t nTime)
{
  uint8_t tempRegValue, measure;
  
  /* Check the parameter */
  s_assert_param(IS_AGC_MEASURE_TIME_US(nTime,s_lXtalFrequency));
  
  /* Reads the AGCCTRL_2 register */
  SpiritSpiReadRegisters(AGCCTRL2_BASE, 1, &tempRegValue);
  
  /* Calculates the measure time value to write in the register */
  measure = (uint8_t)lroundf(log2((float)nTime/1e6 * s_lXtalFrequency/12));
  (measure>15) ? (measure=15):(measure);
  
  /* Mask the MEAS_TIME field and write the new value */
  tempRegValue &= 0xF0;
  tempRegValue |= measure;
  
  /* Sets the AGCCTRL_2 register */
  g_xStatus = SpiritSpiWriteRegisters(AGCCTRL2_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the AGC measure time.
* @param  None.
* @retval uint16_t AGC measure time expressed in us. This parameter will be in the range [0, 393216/F_Xo].
*/
uint16_t SpiritRadioGetAGCMeasureTimeUs(void)
{
  uint8_t measure;
  
  /* Reads the AGCCTRL_2 register */
  g_xStatus = SpiritSpiReadRegisters(AGCCTRL2_BASE, 1, &measure);
  
  /* Mask the MEAS_TIME field */
  measure &= 0x0F;
  
  /* Calculates the measure time value to write in the register */
  return (uint16_t)((12.0/s_lXtalFrequency)*(float)pow(2,measure)*1e6);
  
}


/**
* @brief  Sets the AGC measure time.
* @param  cTime AGC measure time to write in the MEAS_TIME field of AGCCTRL_2 register.
*         This parameter shall be in the range [0:15].
* @retval None.
*/
void SpiritRadioSetAGCMeasureTime(uint8_t cTime)
{
  uint8_t tempRegValue;
  
  /* Check the parameter */
  s_assert_param(IS_AGC_MEASURE_TIME(cTime));
  
  /* Reads the AGCCTRL_2 register */
  SpiritSpiReadRegisters(AGCCTRL2_BASE, 1, &tempRegValue);
  
  /* Mask the MEAS_TIME field and write the new value */
  tempRegValue &= 0xF0;
  tempRegValue |= cTime;
  
  /* Sets the AGCCTRL_2 register */
  g_xStatus = SpiritSpiWriteRegisters(AGCCTRL2_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the AGC measure time.
* @param  None.
* @retval uint8_t AGC measure time read from the MEAS_TIME field of AGCCTRL_2 register.
*         This parameter will be in the range [0:15].
*/
uint8_t SpiritRadioGetAGCMeasureTime(void)
{
  uint8_t tempRegValue;
  
  /* Reads the AGCCTRL_2 register, mask the MEAS_TIME field and return the value  */
  g_xStatus = SpiritSpiReadRegisters(AGCCTRL2_BASE, 1, &tempRegValue);
  
  return (tempRegValue & 0x0F);
  
}


/**
* @brief  Sets the AGC hold time.
* @param  cTime AGC hold time expressed in us. This parameter shall be in the range[0, 756/F_Xo].
* @retval None.
*/
void SpiritRadioSetAGCHoldTimeUs(uint8_t cTime)
{
  uint8_t tempRegValue, hold;
  
  /* Check the parameter */
  s_assert_param(IS_AGC_HOLD_TIME_US(cTime,s_lXtalFrequency));
  
  /* Reads the AGCCTRL_0 register */
  SpiritSpiReadRegisters(AGCCTRL0_BASE, 1, &tempRegValue);
  
  /* Calculates the hold time value to write in the register */
  hold = (uint8_t)lroundf(((float)cTime/1e6 * s_lXtalFrequency)/12);
  (hold>63) ? (hold=63):(hold);
  
  /* Mask the HOLD_TIME field and write the new value */
  tempRegValue &= 0xC0;
  tempRegValue |= hold;
  
  /* Sets the AGCCTRL_0 register */
  g_xStatus = SpiritSpiWriteRegisters(AGCCTRL0_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the AGC hold time.
* @param  None.
* @retval uint8_t AGC hold time expressed in us. This parameter will be in the range:
*         [0, 756/F_Xo].
*/
uint8_t SpiritRadioGetAGCHoldTimeUs(void)
{
  uint8_t tempRegValue;
  
  /* Reads the AGCCTRL_0 register */
  g_xStatus = SpiritSpiReadRegisters(AGCCTRL0_BASE, 1, &tempRegValue);
  
  /* Mask the HOLD_TIME field */
  tempRegValue &= 0x3F;
  
  /* Calculates the hold time value and return it */
  return (uint8_t)lroundf ((12.0/s_lXtalFrequency)*(tempRegValue*1e6));
  
}


/**
* @brief  Sets the AGC hold time.
* @param  cTime AGC hold time to write in the HOLD_TIME field of AGCCTRL_0 register.
*         This parameter shall be in the range [0:63].
* @retval None.
*/
void SpiritRadioSetAGCHoldTime(uint8_t cTime)
{
  uint8_t tempRegValue;
  
  /* Check the parameter */
  s_assert_param(IS_AGC_HOLD_TIME(cTime));
  
  /* Reads the AGCCTRL_0 register */
  SpiritSpiReadRegisters(AGCCTRL0_BASE, 1, &tempRegValue);
  
  /* Mask the HOLD_TIME field and write the new value */
  tempRegValue &= 0xC0;
  tempRegValue |= cTime;
  
  /* Sets the AGCCTRL_0 register */
  g_xStatus = SpiritSpiWriteRegisters(AGCCTRL0_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the AGC hold time.
* @param  None.
* @retval uint8_t AGC hold time read from the HOLD_TIME field of AGCCTRL_0 register.
*         This parameter will be in the range [0:63].
*/
uint8_t SpiritRadioGetAGCHoldTime(void)
{
  uint8_t tempRegValue;
  
  /* Reads the AGCCTRL_0 register, mask the MEAS_TIME field and return the value  */
  g_xStatus = SpiritSpiReadRegisters(AGCCTRL0_BASE, 1, &tempRegValue);
  
  return (tempRegValue & 0x3F);
  
}


/**
* @brief  Sets the AGC high threshold.
* @param  cHighThreshold AGC high threshold to write in the THRESHOLD_HIGH field of AGCCTRL_1 register.
*         This parameter shall be in the range [0:15].
* @retval None.
*/
void SpiritRadioSetAGCHighThreshold(uint8_t cHighThreshold)
{
  uint8_t tempRegValue;
  
  /* Check the parameter */
  s_assert_param(IS_AGC_THRESHOLD(cHighThreshold));
  
  /* Reads the AGCCTRL_1 register */
  SpiritSpiReadRegisters(AGCCTRL1_BASE, 1, &tempRegValue);
  
  /* Mask the THRESHOLD_HIGH field and write the new value */
  tempRegValue &= 0x0F;
  tempRegValue |= cHighThreshold<<4;
  
  /* Sets the AGCCTRL_1 register */
  g_xStatus = SpiritSpiWriteRegisters(AGCCTRL1_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the AGC high threshold.
* @param  None.
* @retval uint8_t AGC high threshold read from the THRESHOLD_HIGH field of AGCCTRL_1 register.
*         This parameter will be in the range [0:15].
*/
uint8_t SpiritRadioGetAGCHighThreshold(void)
{
  uint8_t tempRegValue;
  
  /* Reads the AGCCTRL_1 register, mask the THRESHOLD_HIGH field and return the value */
  g_xStatus = SpiritSpiReadRegisters(AGCCTRL1_BASE, 1, &tempRegValue);
  
  return ((tempRegValue & 0xF0)>>4);
  
}


/**
* @brief  Sets the AGC low threshold.
* @param  cLowThreshold AGC low threshold to write in the THRESHOLD_LOW field of AGCCTRL_1 register.
*         This parameter shall be in the range [0:15].
* @retval None.
*/
void SpiritRadioSetAGCLowThreshold(uint8_t cLowThreshold)
{
  uint8_t tempRegValue;
  
  /* Check the parameter */
  s_assert_param(IS_AGC_THRESHOLD(cLowThreshold));
  
  /* Reads the AGCCTRL_1 register */
  SpiritSpiReadRegisters(AGCCTRL1_BASE, 1, &tempRegValue);
  
  /* Mask the THRESHOLD_LOW field and write the new value */
  tempRegValue &= 0xF0;
  tempRegValue |= cLowThreshold;
  
  /* Sets the AGCCTRL_1 register */
  g_xStatus = SpiritSpiWriteRegisters(AGCCTRL1_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the AGC low threshold.
* @param  None.
* @retval uint8_t AGC low threshold read from the THRESHOLD_LOW field of AGCCTRL_1 register.
*         This parameter will be in the range [0:15].
*/
uint8_t SpiritRadioGetAGCLowThreshold(void)
{
  uint8_t tempRegValue;
  
  /* Reads the AGCCTRL_1 register, mask the THRESHOLD_LOW field and return the value  */
  g_xStatus = SpiritSpiReadRegisters(AGCCTRL1_BASE, 1, &tempRegValue);
  
  return (tempRegValue & 0x0F);
  
}


/**
* @brief  Sets the clock recovery algorithm.
* @param  xMode the Clock Recovery mode. This parameter can be one of the values defined in @ref ClkRecMode :
*         @arg CLK_REC_PLL     PLL alogrithm for clock recovery
*         @arg CLK_REC_DLL     DLL alogrithm for clock recovery
* @retval None.
*/
void SpiritRadioSetClkRecMode(ClkRecMode xMode)
{
  uint8_t tempRegValue;
  
  /* Check the parameter */
  s_assert_param(IS_CLK_REC_MODE(xMode));
  
  /* Reads the FDEV_0 register */
  SpiritSpiReadRegisters(FDEV0_BASE, 1, &tempRegValue);
  
  /* Mask the CLOCK_REC_ALGO_SEL field and write the new value */
  tempRegValue &= 0xF7;
  tempRegValue |= (uint8_t)xMode;
  
  /* Sets the FDEV_0 register */
  g_xStatus = SpiritSpiWriteRegisters(FDEV0_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the Clock Recovery working mode.
* @param  None.
* @retval ClkRecMode Clock Recovery mode. This parameter can be one of the values defined in @ref ClkRecMode :
*         @arg CLK_REC_PLL     PLL alogrithm for clock recovery
*         @arg CLK_REC_DLL     DLL alogrithm for clock recovery
*/
ClkRecMode SpiritRadioGetClkRecMode(void)
{
  uint8_t tempRegValue;
  
  /* Reads the FDEV_0 register, mask the CLOCK_REC_ALGO_SEL field and return the value */
  g_xStatus = SpiritSpiReadRegisters(FDEV0_BASE, 1, &tempRegValue);
  
  return (ClkRecMode)(tempRegValue & 0x08);
  
}


/**
* @brief  Sets the clock recovery proportional gain.
* @param  cPGain the Clock Recovery proportional gain to write in the CLK_REC_P_GAIN field of CLOCKREC register.
*         It represents is log2 value of the clock recovery proportional gain.
*          This parameter shall be in the range [0:7].
* @retval None.
*/
void SpiritRadioSetClkRecPGain(uint8_t cPGain)
{
  uint8_t tempRegValue;
  
  /* Check the parameter */
  s_assert_param(IS_CLK_REC_P_GAIN(cPGain));
  
  /* Reads the CLOCKREC register */
  SpiritSpiReadRegisters(CLOCKREC_BASE, 1, &tempRegValue);
  
  /* Mask the CLK_REC_P_GAIN field and write the new value */
  tempRegValue &= 0x1F;
  tempRegValue |= (cPGain<<5);
  
  /* Sets the CLOCKREC register */
  g_xStatus = SpiritSpiWriteRegisters(CLOCKREC_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the log2 of the clock recovery proportional gain.
* @param  None.
* @retval uint8_t Clock Recovery proportional gain read from the CLK_REC_P_GAIN field of CLOCKREC register.
*         This parameter will be in the range [0:7].
*/
uint8_t SpiritRadioGetClkRecPGain(void)
{
  uint8_t tempRegValue;
  
  /* Reads the CLOCKREC register, mask the CLK_REC_P_GAIN field and return the value  */
  g_xStatus = SpiritSpiReadRegisters(CLOCKREC_BASE, 1, &tempRegValue);
  
  return ((tempRegValue & 0xEF)>>5);
  
}


/**
* @brief  Sets the clock recovery integral gain.
* @param  cIGain the Clock Recovery integral gain to write in the CLK_REC_I_GAIN field of CLOCKREC register.
*         This parameter shall be in the range [0:15].
* @retval None.
*/
void SpiritRadioSetClkRecIGain(uint8_t cIGain)
{
  uint8_t tempRegValue;
  
  /* Check the parameter */
  s_assert_param(IS_CLK_REC_I_GAIN(cIGain));
  
  /* Reads the CLOCKREC register */
  SpiritSpiReadRegisters(CLOCKREC_BASE, 1, &tempRegValue);
  
  /* Mask the CLK_REC_P_GAIN field and write the new value */
  tempRegValue &= 0xF0;
  tempRegValue |= cIGain;
  
  /* Sets the CLOCKREC register */
  g_xStatus = SpiritSpiWriteRegisters(CLOCKREC_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the clock recovery integral gain.
* @param  None.
* @retval uint8_t Clock Recovery integral gain read from the
*         CLK_REC_I_GAIN field of CLOCKREC register.
*         This parameter will be in the range [0:15].
*/
uint8_t SpiritRadioGetClkRecIGain(void)
{
  uint8_t tempRegValue;
  
  /* Reads the CLOCKREC register, mask the CLK_REC_I_GAIN field and return the value */
  g_xStatus = SpiritSpiReadRegisters(CLOCKREC_BASE, 1, &tempRegValue);
  
  return (tempRegValue & 0x0F);
  
}


/**
* @brief  Sets the postfilter length for clock recovery algorithm.
* @param  xLength the postfilter length in symbols. This parameter can be one of the values defined in @ref PstFltLength :
*         @arg PSTFLT_LENGTH_8     Postfilter length is 8 symbols
*         @arg PSTFLT_LENGTH_16    Postfilter length is 16 symbols
* @retval None.
*/
void SpiritRadioSetClkRecPstFltLength(PstFltLength xLength)
{
  uint8_t tempRegValue;
  
  /* Check the parameter */
  s_assert_param(IS_PST_FLT_LENGTH(xLength));
  
  /* Reads the CLOCKREC register */
  SpiritSpiReadRegisters(CLOCKREC_BASE, 1, &tempRegValue);
  
  /* Mask the PSTFLT_LEN field and write the new value */
  tempRegValue &= 0xEF;
  tempRegValue |= (uint8_t)xLength;
  
  /* Sets the CLOCKREC register */
  g_xStatus = SpiritSpiWriteRegisters(CLOCKREC_BASE, 1, &tempRegValue);
  
}


/**
* @brief  Returns the postfilter length for clock recovery algorithm.
* @param  None.
* @retval PstFltLength Postfilter length in symbols. This parameter can be one of the values defined in @ref PstFltLength :
*         @arg PSTFLT_LENGTH_8     Postfilter length is 8 symbols
*         @arg PSTFLT_LENGTH_16    Postfilter length is 16 symbols
*/
PstFltLength SpiritRadioGetClkRecPstFltLength(void)
{
  uint8_t tempRegValue;
  
  /* Reads the CLOCKREC register, mask the PSTFLT_LEN field and return the value */
  g_xStatus = SpiritSpiReadRegisters(CLOCKREC_BASE, 1, &tempRegValue);
  
  return (PstFltLength)(tempRegValue & 0x10);
  
}


/**
* @brief  Enables or Disables the received data blanking when the CS is under the threshold.
* @param  xNewState new state of this mode.
*         This parameter can be: S_ENABLE or S_DISABLE .
* @retval None.
*/
void SpiritRadioCsBlanking(SpiritFunctionalState xNewState)
{
  uint8_t tempRegValue;
  
  /* Check the parameters */
  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(xNewState));
  
  /* Reads the ANT_SELECT_CONF_BASE and mask the CS_BLANKING BIT field */
  SpiritSpiReadRegisters(ANT_SELECT_CONF_BASE, 1, &tempRegValue);
  
  if(xNewState == S_ENABLE)
  {
    tempRegValue |= ANT_SELECT_CS_BLANKING_MASK;
  }
  else
  {
    tempRegValue &= (~ANT_SELECT_CS_BLANKING_MASK);
  }
  
  /* Writes the new value in the ANT_SELECT_CONF register */
  g_xStatus = SpiritSpiWriteRegisters(ANT_SELECT_CONF_BASE, 1, &tempRegValue);
  
  
}

/**
* @brief  Enables or Disables the persistent RX mode.
* @param  xNewState new state of this mode.
*         This parameter can be: S_ENABLE or S_DISABLE .
* @retval None.
*/
void SpiritRadioPersistenRx(SpiritFunctionalState xNewState)
{
  uint8_t tempRegValue;
  
  /* Check the parameters */
  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(xNewState));
  
  /* Reads the PROTOCOL0_BASE and mask the PROTOCOL0_PERS_RX_MASK bitfield */
  SpiritSpiReadRegisters(PROTOCOL0_BASE, 1, &tempRegValue);
  
  if(xNewState == S_ENABLE)
  {
    tempRegValue |= PROTOCOL0_PERS_RX_MASK;
  }
  else
  {
    tempRegValue &= (~PROTOCOL0_PERS_RX_MASK);
  }
  
  /* Writes the new value in the PROTOCOL0_BASE register */
  g_xStatus = SpiritSpiWriteRegisters(PROTOCOL0_BASE, 1, &tempRegValue);
  
}

/**
* @brief  Enables or Disables the synthesizer reference divider.
* @param  xNewState new state for synthesizer reference divider.
*         This parameter can be: S_ENABLE or S_DISABLE .
* @retval None.
*/
void SpiritRadioSetRefDiv(SpiritFunctionalState xNewState)
{
  uint8_t tempRegValue;
  
  /* Check the parameters */
  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(xNewState));
  
  /* Reads the SYNTH_CONFIG1_BASE and mask the REFDIV bit field */
  SpiritSpiReadRegisters(SYNTH_CONFIG1_BASE, 1, &tempRegValue);

  if(xNewState == S_ENABLE)
  {
    tempRegValue |= 0x80;
  }
  else
  {
    tempRegValue &= 0x7F;
  }
  
  /* Writes the new value in the SYNTH_CONFIG1_BASE register */
  g_xStatus = SpiritSpiWriteRegisters(SYNTH_CONFIG1_BASE, 1, &tempRegValue);
  
}

/**
* @brief  Get the the synthesizer reference divider state.
* @param  void.
* @retval None.
*/
SpiritFunctionalState SpiritRadioGetRefDiv(void)
{
  uint8_t tempRegValue;
  
  g_xStatus = SpiritSpiReadRegisters(SYNTH_CONFIG1_BASE, 1, &tempRegValue);
  
  if(((tempRegValue>>7)&0x1))
  {
    return S_ENABLE;
  }
  else
  {
    return S_DISABLE;
  }
  
}

/**
* @brief  Enables or Disables the synthesizer reference divider.
* @param  xNewState new state for synthesizer reference divider.
*         This parameter can be: S_ENABLE or S_DISABLE .
* @retval None.
*/
void SpiritRadioSetDigDiv(SpiritFunctionalState xNewState)
{
  uint8_t tempRegValue;
  
  /* Check the parameters */
  s_assert_param(IS_SPIRIT_FUNCTIONAL_STATE(xNewState));
  
  /* Reads the XO_RCO_TEST_BASE and mask the PD_CLKDIV bit field */
  SpiritSpiReadRegisters(XO_RCO_TEST_BASE, 1, &tempRegValue);

  if(xNewState == S_ENABLE)
  {
    tempRegValue &= 0xf7;
  }
  else
  {
    
    tempRegValue |= 0x08;
  }
  
  /* Writes the new value in the XO_RCO_TEST_BASE register */
  g_xStatus = SpiritSpiWriteRegisters(XO_RCO_TEST_BASE, 1, &tempRegValue);
  
}

/**
* @brief  Get the the synthesizer reference divider state.
* @param  void.
* @retval None.
*/
SpiritFunctionalState SpiritRadioGetDigDiv(void)
{
  uint8_t tempRegValue;
  
  g_xStatus = SpiritSpiReadRegisters(XO_RCO_TEST_BASE, 1, &tempRegValue);
  
  if(((tempRegValue>>3)&0x1))
  {
    return S_DISABLE;
  }
  else
  {
    return S_ENABLE;
  }
  
}

/**
* @brief  Returns the XTAL frequency.
* @param  void.
* @retval uint32_t XTAL frequency.
*/
uint32_t SpiritRadioGetXtalFrequency(void)
{
  return s_lXtalFrequency; 
}

/**
* @brief  Sets the XTAL frequency.
* @param  uint32_t XTAL frequency.
* @retval void.
*/
void SpiritRadioSetXtalFrequency(uint32_t lXtalFrequency)
{
  s_lXtalFrequency = lXtalFrequency; 
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



/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/

