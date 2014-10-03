/**
 * @file    SPIRIT_Timer.c
 * @author  High End Analog & RF BU - AMS / ART Team IMS-Systems Lab
 * @version V3.0.1
 * @date    November 19, 2012
 * @brief   Configuration and management of SPIRIT timers.
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
#include "SPIRIT_Timer.h"
#include "SPIRIT_Radio.h"
#include "MCU_Interface.h"




/**
 * @addtogroup SPIRIT_Libraries
 * @{
 */


/**
 * @addtogroup SPIRIT_Timer
 * @{
 */


/**
 * @defgroup Timer_Private_TypesDefinitions             Timer Private Types Definitions
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Timer_Private_Defines                      Timer Private Defines
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Timer_Private_Macros                       Timer Private Macros
 * @{
 */


/**
 *@}
 */


/**
 * @defgroup Timer_Private_Variables                    Timer Private Variables
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Timer_Private_FunctionPrototypes            Timer Private Function Prototypes
 * @{
 */

/**
 *@}
 */


/**
 * @defgroup Timer_Private_Functions                    Timer Private Functions
 * @{
 */

/**
 * @brief  Enables or Disables the LDCR mode.
 * @param  xNewState new state for LDCR mode.
 *         This parameter can be: S_ENABLE or S_DISABLE.
 * @retval None.
 */
void SpiritTimerLdcrMode(SpiritFunctionalState xNewState)
{
  uint8_t tempRegValue;

  /* Reads the register value */
  g_xStatus = SpiritSpiReadRegisters(PROTOCOL2_BASE, 1, &tempRegValue);

  /* Mask the read value to enable or disable the LDC mode */
  if(xNewState==S_ENABLE)
  {
    tempRegValue |= PROTOCOL2_LDC_MODE_MASK;
  }
  else
  {
    tempRegValue &= ~PROTOCOL2_LDC_MODE_MASK;
  }

  /* Writes the register to Enable or Disable the LDCR mode */
  g_xStatus = SpiritSpiWriteRegisters(PROTOCOL2_BASE, 1, &tempRegValue);

}


/**
 * @brief  Enables or Disables the LDCR timer reloading with the value stored in the LDCR_RELOAD registers.
 * @param  xNewState new state for LDCR reloading.
 *         This parameter can be: S_ENABLE or S_DISABLE.
 * @retval None.
 */
void SpiritTimerLdcrAutoReload(SpiritFunctionalState xNewState)
{
  uint8_t tempRegValue;

  /* Reads the register value */
  g_xStatus = SpiritSpiReadRegisters(PROTOCOL1_BASE, 1, &tempRegValue);

  /* Mask te read value to enable or disable the reload on sync mode */
  if(xNewState==S_ENABLE)
  {
    tempRegValue |= PROTOCOL1_LDC_RELOAD_ON_SYNC_MASK;
  }
  else
  {
    tempRegValue &= ~PROTOCOL1_LDC_RELOAD_ON_SYNC_MASK;
  }

  /* Writes the register to Enable or Disable the Auto Reload */
  g_xStatus = SpiritSpiWriteRegisters(PROTOCOL1_BASE, 1, &tempRegValue);

}


/**
 * @brief  Returns the LDCR timer reload bit.
 * @param  None.
 * @retval SpiritFunctionalState: value of the reload bit.
 */
SpiritFunctionalState SpiritTimerLdcrGetAutoReload(void)
{
  uint8_t tempRegValue;

  /* Reads the register value */
  g_xStatus = SpiritSpiReadRegisters(PROTOCOL1_BASE, 1, &tempRegValue);

  return (SpiritFunctionalState)(tempRegValue & 0x80);

}

/**
 * @brief  Sets the RX timeout timer initialization registers with the values of COUNTER and PRESCALER according to the formula: Trx=PRESCALER*COUNTER*Tck.
 *         Remember that it is possible to have infinite RX_Timeout writing 0 in the RX_Timeout_Counter and/or RX_Timeout_Prescaler registers.
 * @param  cCounter value for the timer counter.
 *         This parameter must be an uint8_t.
 * @param  cPrescaler value for the timer prescaler.
 *         This parameter must be an uint8_t.
 * @retval None.
 */
void SpiritTimerSetRxTimeout(uint8_t cCounter , uint8_t cPrescaler)
{
  uint8_t tempRegValue[2]={cPrescaler,cCounter};

  /* Writes the prescaler and counter value for RX timeout in the corresponding register */
  g_xStatus = SpiritSpiWriteRegisters(TIMERS5_RX_TIMEOUT_PRESCALER_BASE, 2, tempRegValue);

}


/**
 * @brief  Sets the RX timeout timer counter and prescaler from the desired value in ms. it is possible to fix the RX_Timeout to
 *         a minimum value of 50.417us to a maximum value of about 3.28 s.
 * @param  fDesiredMsec desired timer value.
 *         This parameter must be a float.
 * @retval None
 */

void SpiritTimerSetRxTimeoutMs(float fDesiredMsec)
{
  uint8_t tempRegValue[2];

  /* Computes the counter and prescaler value */
  SpiritTimerComputeRxTimeoutValues(fDesiredMsec , &tempRegValue[1] , &tempRegValue[0]);

  /* Writes the prescaler and counter value for RX timeout in the corresponding register */
  g_xStatus = SpiritSpiWriteRegisters(TIMERS5_RX_TIMEOUT_PRESCALER_BASE, 2, tempRegValue);

}


/**
 * @brief  Sets the RX timeout timer counter. If it is equal to 0 the timeout is infinite.
 * @param  cCounter value for the timer counter.
 *         This parameter must be an uint8_t.
 * @retval None.
 */
void SpiritTimerSetRxTimeoutCounter(uint8_t cCounter)
{
  /* Writes the counter value for RX timeout in the corresponding register */
  g_xStatus = SpiritSpiWriteRegisters(TIMERS4_RX_TIMEOUT_COUNTER_BASE, 1, &cCounter);

}


/**
 * @brief  Sets the RX timeout timer prescaler. If it is equal to 0 the timeout is infinite.
 * @param  cPrescaler value for the timer prescaler.
 *         This parameter must be an uint8_t.
 * @retval None
 */
void SpiritTimerSetRxTimeoutPrescaler(uint8_t cPrescaler)
{
  /* Writes the prescaler value for RX timeout in the corresponding register */
  g_xStatus = SpiritSpiWriteRegisters(TIMERS5_RX_TIMEOUT_PRESCALER_BASE, 1, &cPrescaler);

}


/**
 * @brief  Returns the RX timeout timer.
 * @param  pfTimeoutMsec pointer to the variable in which the timeout expressed in milliseconds has to be stored.
 *         If the returned value is 0, it means that the RX_Timeout is infinite.
 *         This parameter must be a float*.
 * @param  pcCounter pointer to the variable in which the timer counter has to be stored.
 *         This parameter must be an uint8_t*.
 * @param  pcPrescaler pointer to the variable in which the timer prescaler has to be stored.
 *         This parameter must be an uint8_t*.
 * @retval None.
 */
void SpiritTimerGetRxTimeout(float* pfTimeoutMsec, uint8_t* pcCounter , uint8_t* pcPrescaler)
{
  uint8_t tempRegValue[2];

  /* Reads the RX timeout registers value */
  g_xStatus = SpiritSpiReadRegisters(TIMERS5_RX_TIMEOUT_PRESCALER_BASE, 2, tempRegValue);

  /* Returns values */
  (*pcPrescaler) = tempRegValue[0];
  (*pcCounter) = tempRegValue[1];
    
  float nXtalFrequency = (float)SpiritRadioGetXtalFrequency();
  if(nXtalFrequency>26000000) {
    nXtalFrequency /= 2.0;
  }
  nXtalFrequency /= 1000.0;
  *pfTimeoutMsec = (float)((tempRegValue[0]+1)*tempRegValue[1]*(1210.0/nXtalFrequency));
  

}


/**
 * @brief  Sets the LDCR wake up timer initialization registers with the values of
 *         COUNTER and PRESCALER according to the formula: Twu=(PRESCALER +1)*(COUNTER+1)*Tck, where
 *         Tck = 28.818 us. The minimum vale of the wakeup timeout is 28.818us (PRESCALER and
 *         COUNTER equals to 0) and the maximum value is about 1.89 s (PRESCALER anc COUNTER equals
 *         to 255).
 * @param  cCounter value for the timer counter.
 *         This parameter must be an uint8_t.
 * @param  cPrescaler value for the timer prescaler.
 *         This parameter must be an uint8_t.
 * @retval None.
 */
void SpiritTimerSetWakeUpTimer(uint8_t cCounter , uint8_t cPrescaler)
{
  uint8_t tempRegValue[2]={cPrescaler,cCounter};

  /* Writes the counter and prescaler value of wake-up timer in the corresponding register */
  g_xStatus = SpiritSpiWriteRegisters(TIMERS3_LDC_PRESCALER_BASE, 2, tempRegValue);

}


/**
 * @brief  Sets the LDCR wake up timer counter and prescaler from the desired value in ms,
 *         according to the formula: Twu=(PRESCALER +1)*(COUNTER+1)*Tck, where Tck = 28.818 us.
 *         The minimum vale of the wakeup timeout is 28.818us (PRESCALER and COUNTER equals to 0)
 *         and the maximum value is about 1.89 s (PRESCALER anc COUNTER equals to 255).
 * @param  fDesiredMsec desired timer value.
 *         This parameter must be a float.
 * @retval None.
 */
void SpiritTimerSetWakeUpTimerMs(float fDesiredMsec)
{
  uint8_t tempRegValue[2];

  /* Computes counter and prescaler */
  SpiritTimerComputeWakeUpValues(fDesiredMsec , &tempRegValue[1] , &tempRegValue[0]);

  /* Writes the counter and prescaler value of wake-up timer in the corresponding register */
  g_xStatus = SpiritSpiWriteRegisters(TIMERS3_LDC_PRESCALER_BASE, 2, tempRegValue);

}


/**
 * @brief  Sets the LDCR wake up timer counter. Remember that this value is incresead by one in the Twu calculation.
 *         Twu=(PRESCALER +1)*(COUNTER+1)*Tck, where Tck = 28.818 us
 * @param  cCounter value for the timer counter.
 *         This parameter must be an uint8_t.
 * @retval None.
 */
void SpiritTimerSetWakeUpTimerCounter(uint8_t cCounter)
{
  /* Writes the counter value for Wake_Up timer in the corresponding register */
  g_xStatus = SpiritSpiWriteRegisters(TIMERS2_LDC_COUNTER_BASE, 1, &cCounter);

}


/**
 * @brief  Sets the LDCR wake up timer prescaler. Remember that this value is incresead by one in the Twu calculation.
 *         Twu=(PRESCALER +1)*(COUNTER+1)*Tck, where Tck = 28.818 us
 * @param  cPrescaler value for the timer prescaler.
 *         This parameter must be an uint8_t.
 * @retval None.
 */
void SpiritTimerSetWakeUpTimerPrescaler(uint8_t cPrescaler)
{
  /* Writes the prescaler value for Wake_Up timer in the corresponding register */
  g_xStatus = SpiritSpiWriteRegisters(TIMERS3_LDC_PRESCALER_BASE, 1, &cPrescaler);

}


/**
 * @brief  Returns the LDCR wake up timer, according to the formula: Twu=(PRESCALER +1)*(COUNTER+1)*Tck, where Tck = 28.818 us.
 * @param  pfWakeUpMsec pointer to the variable in which the wake-up time expressed in milliseconds has to be stored.
 *         This parameter must be a float*.
 * @param  pcCounter pointer to the variable in which the timer counter has to be stored.
 *         This parameter must be an uint8_t*.
 * @param  pcPrescaler pointer to the variable in which the timer prescaler has to be stored.
 *         This parameter must be an uint8_t*.
 * @retval None.
 */
void SpiritTimerGetWakeUpTimer(float* pfWakeUpMsec, uint8_t* pcCounter , uint8_t* pcPrescaler)
{
  uint8_t tempRegValue[2];

  /* Reads the Wake_Up timer registers value */
  g_xStatus = SpiritSpiReadRegisters(TIMERS3_LDC_PRESCALER_BASE, 2, tempRegValue);

  /* Returns values */
  (*pcPrescaler)=tempRegValue[0];
  (*pcCounter)=tempRegValue[1];
  *pfWakeUpMsec = (float)((((*pcPrescaler)+2)*((*pcCounter)+2)*(1000.0/34.7)));

}


/**
 * @brief  Sets the LDCR wake up timer reloading registers with the values of
 *         COUNTER and PRESCALER according to the formula: Twu=(PRESCALER +1)*(COUNTER+1)*Tck, where
 *         Tck = 28.818 us. The minimum vale of the wakeup timeout is 28.818us (PRESCALER and
 *         COUNTER equals to 0) and the maximum value is about 1.89 s (PRESCALER anc COUNTER equals
 *         to 255).
 * @param  cCounter reload value for the timer counter.
 *         This parameter must be an uint8_t.
 * @param  cPrescaler reload value for the timer prescaler.
 *         This parameter must be an uint8_t.
 * @retval None.
 */
void SpiritTimerSetWakeUpTimerReload(uint8_t cCounter , uint8_t cPrescaler)
{
  uint8_t tempRegValue[2]={cPrescaler,cCounter};

  /* Writes the counter and prescaler value of reload wake-up timer in the corresponding register */
  g_xStatus = SpiritSpiWriteRegisters(TIMERS1_LDC_RELOAD_PRESCALER_BASE, 2, tempRegValue);

}


/**
 * @brief  Sets the LDCR wake up reload timer counter and prescaler from the desired value in ms,
 *         according to the formula: Twu=(PRESCALER +1)*(COUNTER+1)*Tck, where Tck = 28.818 us.
 *         The minimum vale of the wakeup timeout is 28.818us (PRESCALER and COUNTER equals to 0)
 *         and the maximum value is about 1.89 s (PRESCALER anc COUNTER equals to 255).
 * @param  fDesiredMsec desired timer value.
 *         This parameter must be a float.
 * @retval None.
 */
void SpiritTimerSetWakeUpTimerReloadMs(float fDesiredMsec)
{
  uint8_t tempRegValue[2];

  /* Computes counter and prescaler */
  SpiritTimerComputeWakeUpValues(fDesiredMsec , &tempRegValue[1] , &tempRegValue[0]);

  /* Writes the counter and prescaler value of reload wake-up timer in the corresponding register */
  g_xStatus = SpiritSpiWriteRegisters(TIMERS1_LDC_RELOAD_PRESCALER_BASE, 2, tempRegValue);

}


/**
 * @brief  Sets the LDCR wake up timer reload counter. Remember that this value is incresead by one in the Twu calculation.
 *         Twu=(PRESCALER +1)*(COUNTER+1)*Tck, where Tck = 28.818 us
 * @param  cCounter value for the timer counter.
 *         This parameter must be an uint8_t.
 * @retval None
 */
void SpiritTimerSetWakeUpTimerReloadCounter(uint8_t cCounter)
{
  /* Writes the counter value for reload Wake_Up timer in the corresponding register */
  g_xStatus = SpiritSpiWriteRegisters(TIMERS0_LDC_RELOAD_COUNTER_BASE, 1, &cCounter);

}


/**
 * @brief  Sets the LDCR wake up timer reload prescaler. Remember that this value is incresead by one in the Twu calculation.
 *         Twu=(PRESCALER +1)*(COUNTER+1)*Tck, where Tck = 28.818 us
 * @param  cPrescaler value for the timer prescaler.
 *         This parameter must be an uint8_t.
 * @retval None
 */
void SpiritTimerSetWakeUpTimerReloadPrescaler(uint8_t cPrescaler)
{
  /* Writes the prescaler value for reload Wake_Up timer in the corresponding register */
  g_xStatus = SpiritSpiWriteRegisters(TIMERS1_LDC_RELOAD_PRESCALER_BASE, 1, &cPrescaler);

}


/**
 * @brief  Returns the LDCR wake up reload timer, according to the formula: Twu=(PRESCALER +1)*(COUNTER+1)*Tck, where Tck = 28.818 us.
 * @param  pfWakeUpReloadMsec pointer to the variable in which the wake-up reload time expressed in milliseconds has to be stored.
 *         This parameter must be a float*.
 * @param  pcCounter pointer to the variable in which the timer counter has to be stored.
 *         This parameter must be an uint8_t*.
 * @param  pcPrescaler pointer to the variable in which the timer prescaler has to be stored.
 *         This parameter must be an uint8_t*.
 * @retval None.
 */
void SpiritTimerGetWakeUpTimerReload(float* pfWakeUpReloadMsec, uint8_t* pcCounter , uint8_t* pcPrescaler)
{
  uint8_t tempRegValue[2];

  /* Reads the reload Wake_Up timer registers value */
  g_xStatus = SpiritSpiReadRegisters(TIMERS1_LDC_RELOAD_PRESCALER_BASE, 2, tempRegValue);

  /* Returns values */
  (*pcPrescaler)=tempRegValue[0];
  (*pcCounter)=tempRegValue[1];
  *pfWakeUpReloadMsec = (float)((((*pcPrescaler)+2)*((*pcCounter)+2)*(1000.0/34.7)));

}


/**
 * @brief  Computes the values of the wakeup timer counter and prescaler from the user time expressed in millisecond.
 *         The prescaler and the counter values are computed maintaining the prescaler value as
 *         small as possible in order to obtain the best resolution, and in the meantime minimizing the error.
 * @param  fDesiredMsec desired wakeup timeout in millisecs.
 *         This parameter must be a float. Since the counter and prescaler are 8 bit registers the maximum
 *         reachable value is maxTime = fTclk x 256 x 256.
 * @param  pcCounter pointer to the variable in which the value for the wakeup timer counter has to be stored.
 *         This parameter must be a uint8_t*.
 * @param  pcPrescaler pointer to the variable in which the value for the wakeup timer prescaler has to be stored.
 *         This parameter must be an uint8_t*.
 * @retval None
 */

void SpiritTimerComputeWakeUpValues(float fDesiredMsec , uint8_t* pcCounter , uint8_t* pcPrescaler)
{
  uint8_t b0, a0;
  uint32_t n;
  int32_t err, err_min;
  
  /* If the desired value is over the maximum limit, the counter and the
  prescaler are settled to their maximum values, and doesn't execute the routine */
  if(fDesiredMsec>1903.0)
  {
    *pcCounter = 0xFF;
    *pcPrescaler = 0xFF;
    return;
  }
  else
  {
    n = (uint32_t)(fDesiredMsec*34.7);
    err_min = n;
    /* These are the initial values for the prescaler and the counter, where the prescaler
    is settled to the minimum value and the counter accordingly. In order to avoid a zero
    division for the counter the prescaler is increased by one. Then because the wakeup timeout
    is calculated as: Twu=(PRESCALER +1)*(COUNTER+1)*Tck the counter and the prescaler are decreased by one.*/
    *pcPrescaler = a0 = (n/0xFF);
     if(a0==0)
      *pcCounter = b0 = 0;
    else
      *pcCounter = b0 = (n / *pcPrescaler)-2;
    
    /* Iterative cycle to minimize the error */
    for (; ; (*pcPrescaler)++)
    {
      *pcCounter = ((n/(*pcPrescaler+2))-2);
      err = (((uint32_t)(*pcPrescaler)+0) * ((uint32_t)*pcCounter)+0) - (uint32_t)n;
      if ((uint32_t)S_ABS(err) > (uint32_t)(*pcPrescaler / 2))
      {
        (*pcCounter)++;
        err = (((uint32_t)(*pcPrescaler)+0) * ((uint32_t)*pcCounter)+0) - (uint32_t)n;
      }
      if (S_ABS(err) < S_ABS(err_min))
      {
        err_min = err;
        a0 = *pcPrescaler;
        b0 = *pcCounter;
        if (err == 0) 
        {
          break;
        }
      }
      if(*pcPrescaler == 0xFF) 
      {
        break;
      }
    }
    if(a0==0)
      a0=1;
    if(b0==0 || b0==1)
      b0=2;
    
    *pcPrescaler = a0;
    *pcCounter = b0-1;
  }

}

/**
 * @brief  Computes the values of the rx_timeout timer counter and prescaler from the user time expressed in millisecond.
 *         The prescaler and the counter values are computed maintaining the prescaler value as
 *         small as possible in order to obtain the best resolution, and in the meantime minimizing the error.
 * @param  fDesiredMsec desired rx_timeout in millisecs.
 *         This parameter must be a float. Since the counter and prescaler are 8 bit registers the maximum
 *         reachable value is maxTime = fTclk x 255 x 255.
 * @param  pcCounter pointer to the variable in which the value for the rx_timeout counter has to be stored.
 *         This parameter must be a uint8_t*.
 * @param  pcPrescaler pointer to the variable in which the value for the rx_timeout prescaler has to be stored.
 *         This parameter must be an uint8_t*.
 * @retval None
 */
void SpiritTimerComputeRxTimeoutValues(float fDesiredMsec , uint8_t* pcCounter , uint8_t* pcPrescaler)
{
  uint32_t nXtalFrequency = SpiritRadioGetXtalFrequency();
  if(nXtalFrequency>26000000) {
    nXtalFrequency >>= 1;
  }
  
  /* If the desired value is over the maximum limit, the counter and the
  prescaler are settled to their maximum values, and doesn't execute the routine */
  if(fDesiredMsec>3291.0 && nXtalFrequency==24000000 || fDesiredMsec>3159.0 && nXtalFrequency==25000000 || fDesiredMsec>3038.0 && nXtalFrequency==26000000)
  {
    *pcCounter = 0xFF;
    *pcPrescaler = 0xFF;
    return;
  }
  else
  {
    float FPeriod = 1210.0 / (nXtalFrequency/1000000);

    uint8_t b0, a0;
    uint32_t n = (uint32_t)((fDesiredMsec*1000)/FPeriod);
    int32_t err, err_min;
    
    err_min = n;
    /* These are the initial values for the prescaler and the counter, where the prescaler
    is settled to the minimum value and the counter accordingly. In order to avoid a zero
    division for the counter the prescaler is increased by one.*/
    
    *pcPrescaler = a0 = (uint8_t)((n-1)/0xFF);
    if(a0==0)
      *pcCounter = b0 = 0;
    else
      *pcCounter = b0 = (uint8_t)(n / *pcPrescaler)-1;

    for (; ; (*pcPrescaler)++)
    {
      *pcCounter = (uint8_t)(n / *pcPrescaler)-1;
      err = (((uint32_t)(*pcPrescaler)+1) * ((uint32_t)*pcCounter)) - (uint32_t)n;

      if ((uint32_t)S_ABS(err) > (uint32_t)(*pcPrescaler / 2))
      {
        (*pcCounter)++;
        err = (((uint32_t)(*pcPrescaler)+1) * ((uint32_t)*pcCounter)) - (uint32_t)n;
      }
      if (S_ABS(err) < S_ABS(err_min))
      {
        err_min = err;
        a0 = *pcPrescaler;
        b0 = *pcCounter;
        if (err_min == 0) 
        {
          break;
        }
      }
      if(*pcPrescaler == (0xFF-1)) 
      {
        break;
      }
    }

    if(a0==0)
      a0=1;
    if(b0==0)
      b0=1;
    
    *pcPrescaler = a0;
    *pcCounter = b0;
  }
}

/**
 * @brief  Sets the RX timeout stop conditions.
 * @param  xStopCondition new stop condition.
 *         This parameter can be any value of @ref RxTimeoutStopCondition.
 * @retval None
 */
void SpiritTimerSetRxTimeoutStopCondition(RxTimeoutStopCondition xStopCondition)
{
  uint8_t tempRegValue[2];

  /* Check the parameters */
  s_assert_param(IS_RX_TIMEOUT_STOP_CONDITION(xStopCondition));

  /* Reads value on the PKT_FLT_OPTIONS and PROTOCOL2 register */
  g_xStatus = SpiritSpiReadRegisters(PCKT_FLT_OPTIONS_BASE, 2, tempRegValue);

  tempRegValue[0] &= 0xBF;
  tempRegValue[0] |= ((xStopCondition & 0x08)  << 3);

  tempRegValue[1] &= 0x1F;
  tempRegValue[1] |= (xStopCondition << 5);

  /* Writes value on the PKT_FLT_OPTIONS and PROTOCOL2 register */
  g_xStatus = SpiritSpiWriteRegisters(PCKT_FLT_OPTIONS_BASE, 2, tempRegValue);

}

/**
 * @brief  Sends the LDC_RELOAD command to SPIRIT. Reload the LDC timer with the value stored in the LDC_PRESCALER / COUNTER registers.
 * @param  None.
 * @retval None
 */
void SpiritTimerReloadStrobe(void)
{
  /* Sends the CMD_LDC_RELOAD command */
  g_xStatus = SpiritSpiCommandStrobes(COMMAND_LDC_RELOAD);

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



/******************* (C) COPYRIGHT 2012 STMicroelectronics *****END OF FILE****/
