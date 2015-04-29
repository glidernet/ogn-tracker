#include "background.h"
#include <stm32l1xx.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <timers.h>
#include "timer_const.h"
#include "messages.h"


/* -------- defines -------- */
/* -------- variables -------- */
/* Background task queue */
xQueueHandle background_que;
static TimerHandle_t  xBKGNDTimer;

/* ADC constants */
static uint32_t vrefint;

/* ADC measurements */
static int16_t s_volt_vdd;
static int16_t s_volt_vbat;

/* -------- interrupt handlers -------- */
/* -------- functions -------- */

void ADC_Config(uint8_t chn_num)
{
    ADC_InitTypeDef ADC_InitStructure;

    /* ADC1 Configuration ------------------------------------------------------*/
  
    /* Enable ADC1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    /* ADC1 regular channel configuration */
    ADC_RegularChannelConfig(ADC1, chn_num, 1, ADC_SampleTime_192Cycles);

    /* Define delay between ADC1 conversions */
    ADC_DelaySelectionConfig(ADC1, ADC_DelayLength_Freeze);
  
    /* Enable ADC1 Power Down during Delay */
    ADC_PowerDownCmd(ADC1, ADC_PowerDown_Idle_Delay, ENABLE);
  
    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);

    /* Wait until ADC1 ON status */
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET)
    {
    }

    /* Start ADC1 Software Conversion */
    ADC_SoftwareStartConv(ADC1);

    /* Wait until ADC Channel end of conversion */
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
    {
    }
}

void MeasureADCs(int16_t* vdd, int16_t* vbat)
{
    __IO uint16_t ADCdata_vref = 0;
    __IO uint16_t ADCdata_vbat = 0; 

    ADC_Config(ADC_Channel_17); 
    ADCdata_vref = ADC_GetConversionValue(ADC1);
    ADC_DeInit(ADC1);
    
    ADC_Config(ADC_Channel_10); 
    ADCdata_vbat = ADC_GetConversionValue(ADC1);
    ADC_DeInit(ADC1);
    
    if (ADCdata_vref) *vdd = (4095*vrefint)/ADCdata_vref;
    if (ADCdata_vbat) *vbat = (ADCdata_vbat*vrefint)/ADCdata_vref;
    /* VBat is 2 times lower (HW) */
    *vbat *= 2;

    /* Return ADC to idle state */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);    
}

int16_t BKGRD_Get_Volt_VDD()
{
    return s_volt_vdd;
}   

int16_t BKGRD_Get_Volt_VBat()
{
    return s_volt_vbat;
}

xQueueHandle* Get_BackgroundQue()
{
   return &background_que;
}

/**
* @brief  Background timer callback.
* @brief  Trigger background task to perform selected activities.
* @param  Timer handle
* @retval None
*/
void vBKGNDTimerCallback(TimerHandle_t pxTimer)
{
    task_message msg;
    
    msg.msg_data   = 0;
    msg.msg_len    = 0;
    msg.msg_opcode = BKGRD_ADC_MEASURE;
    msg.src_id     = BKGRD_SRC_ID;
    xQueueSend(background_que, &msg, portMAX_DELAY); 
}

/**
* @brief  Configures the Background Task Peripherals.
* @param  None
* @retval None
*/
void Background_Config(void)
{
    /* Enable The HSI (16Mhz) - required by ADC */
    RCC_HSICmd(ENABLE);
    /* Check that HSI oscillator is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);
   
    s_volt_vdd  = -1;
    s_volt_vbat = -1;
    /* Internal STM32L reference voltage typical value */
    /* It could be calibrated if needed */
    vrefint = 1224; /* 1224 mV */
    
    xBKGNDTimer = xTimerCreate("BKGND",
       /* The timer period in ticks. */
       TIMER_MS(10000),
       /* The timer will repeat when expire. */
       pdTRUE,
       /* unique id */
       (void*)BKGND_TASK_TIMER_ID,
       /* Each timer calls the same callback when it expires. */
       vBKGNDTimerCallback
    );
     
    background_que = xQueueCreate(5, sizeof(task_message));
}


/**
* @brief  Main Background Task.
* @param  None
* @retval None
*/
void vTaskBackground(void* pvParameters)
{
    task_message msg;

    xTimerStart(xBKGNDTimer, 0);
    
    for(;;)
    {
        xQueueReceive(background_que, &msg, portMAX_DELAY);
      
        switch (msg.msg_opcode)
        {
            case BKGRD_ADC_MEASURE:
                MeasureADCs(&s_volt_vdd, &s_volt_vbat);
                break;
                
            default:
                break;
        }
    }
}
