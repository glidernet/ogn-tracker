#include "background.h"
#include <stm32l1xx.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <timers.h>
#include "timer_const.h"
#include "messages.h"

/* ------- constants --------*/
const uint16_t ADC_max_val  = 4095;
const uint32_t ts_cal1_temp = 30;
const uint32_t ts_cal2_temp = 110;

/* -------- defines -------- */
/* -------- variables -------- */
/* Background task queue */
xQueueHandle background_que;
static TimerHandle_t  xBKGNDTimer;

/* --- ADC constants --- */
static uint32_t vrefint;
/* Temperature sensor calibration values */
static uint32_t ts_cal1_uV;
static uint32_t ts_cal2_uV;
static int32_t  ts_adc_slope_uV;

/* ADC measurements */
static int16_t s_volt_vdd;
static int16_t s_volt_vbat;
static int16_t s_temp_sens;

/* -------- interrupt handlers -------- */
/* -------- functions -------- */

uint16_t ADC_Config_Measure(uint8_t chn_num)
{
    ADC_InitTypeDef ADC_InitStructure;
    uint16_t    ret_val;
    /* ADC1 Configuration ------------------------------------------------------*/
  
    /* Enable ADC1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
  
    /* ADC1 regular channel configuration */
    ADC_RegularChannelConfig(ADC1, chn_num, 1, ADC_SampleTime_384Cycles);
  
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
    ret_val = ADC_GetConversionValue(ADC1);
    ADC_DeInit(ADC1);
    return ret_val;
}

void MeasureADCs(int16_t* vdd, int16_t* vbat, int16_t* cpu_temp)
{
    __IO uint16_t ADCdata_vref;
    __IO uint16_t ADCdata_vbat; 
    __IO uint16_t ADCdata_tsens;    
    int32_t tsens_diff; 
    
    ADC_TempSensorVrefintCmd(ENABLE);
    
    ADCdata_vref  = ADC_Config_Measure(ADC_Channel_Vrefint);     
    ADCdata_tsens = ADC_Config_Measure(ADC_Channel_TempSensor); 
    ADCdata_vbat  = ADC_Config_Measure(ADC_Channel_10); 
    
    if (ADCdata_vref) *vdd  = (ADC_max_val*vrefint)/ADCdata_vref;
    if (ADCdata_vref) *vbat = (ADCdata_vbat*vrefint)/ADCdata_vref;
    /* VBat is 2 times lower (HW) */
    *vbat *= 2;
    
    /* Get temp. sensor readings in uV and calc. diff. from cal1 value */
    tsens_diff = (1000*(*vdd)*ADCdata_tsens/ADC_max_val) - ts_cal1_uV;    
    *cpu_temp = 10*ts_cal1_temp + 10*tsens_diff/ts_adc_slope_uV;
    
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

int16_t BKGRD_Get_CPU_Temp()
{
    return s_temp_sens;
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
   
    /* Get temperature sensor calibration values */
    /* Convert from ADC readings to uV using 3.0V as VDDA */
    ts_cal1_uV = *(uint16_t*)0x1FF800FA*3*1e6/ADC_max_val; 
    ts_cal2_uV = *(uint16_t*)0x1FF800FE*3*1e6/ADC_max_val;
    /* Calculate ADC slope */
    ts_adc_slope_uV = (ts_cal2_uV-ts_cal1_uV)/(ts_cal2_temp-ts_cal1_temp);
    
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
                MeasureADCs(&s_volt_vdd, &s_volt_vbat, &s_temp_sens);
                break;
                
            default:
                break;
        }
    }
}
