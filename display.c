#include "display.h"
#include <stm32l1xx.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <timers.h>
#include "timer_const.h"
#include "messages.h"

/* -------- defines -------- */

/* ------ GPS LED constants ------ */
/* - GPS LED pin mappings - */
#define GPS_LED_PIN   GPIO_Pin_5
#define GPS_LED_PORT  GPIOC
#define GPS_LED_CLK   RCC_AHBPeriph_GPIOC

/* - GPS LED states - */ 
typedef enum
{
    GPS_LED_NOT_ACTIVE = -1,
    GPS_LED_NO_FIX = 0,
    GPS_LED_FIX = 1,
} GPS_LED_status_t;

/* - GPS LED on/off activity times - */ 
const uint16_t gps_led_times[2][2] = 
{
  /* ON    OFF */
    {1000, 1000}, /* GPS_LED_NO_FIX */
    {100,   900}  /* GPS_LED_FIX    */
};

/* -------- variables -------- */
/* Display task queue */
xQueueHandle        display_que;

/* ------ GPS LED variables ------ */
static TimerHandle_t     xGPSLEDTimer;    /* one timer for controlling all GPS LED states transition */
static GPS_LED_status_t  gps_led_status;  /* current state of GPS LED */

/* -------- interrupt handlers -------- */
/* -------- functions -------- */
xQueueHandle* Get_DisplayQue()
{
   return &display_que;
}

/**
* @brief  GPS LED control function.
* @brief  Function controls GPS LED status pin according to gps_led_times.
* @param  New GPS LED state
* @retval None
*/

void GPS_LED_Start(GPS_LED_status_t new_status)
{
    static uint8_t gps_led_idx = 0; /* index of time index in gps_led_times */
    uint16_t new_tick;              /* new timer timeout value */
    
    /* store new status value in global variable */
    gps_led_status = new_status;
    /* toggle GPS LED according to current index in table */ 
    if (gps_led_idx == 0)  GPIO_SetBits(GPS_LED_PORT, GPS_LED_PIN);
    else                   GPIO_ResetBits(GPS_LED_PORT, GPS_LED_PIN);
    
    /* get new timeout value according to GPS LED state and time index */
    new_tick = gps_led_times[gps_led_status][gps_led_idx++];
    /* restart time index if needed */
    if (gps_led_idx > 1) gps_led_idx = 0;
    /* Reprogram GPS LED timer with new values */
    xTimerChangePeriod(xGPSLEDTimer, new_tick, 0);
    xTimerStart(xGPSLEDTimer, 0);
}

/**
* @brief  GPS LED timer callback.
* @brief  Callback is called when current GPS LED time is finished.
* @param  Timer handle
* @retval None
*/
void vGPSLEDTimerCallback(TimerHandle_t pxTimer)
{
    GPS_LED_Start(gps_led_status);
}

/**
* @brief  Configures the Display Task Peripherals.
* @param  None
* @retval None
*/
void Display_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    gps_led_status = GPS_LED_NOT_ACTIVE;
    
    /* GPIO PC5 - GPS Status*/  
    RCC_AHBPeriphClockCmd(GPS_LED_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Pin   = GPS_LED_PIN;
    GPIO_Init(GPS_LED_PORT, &GPIO_InitStructure);
   
    GPIO_SetBits(GPS_LED_PORT, GPS_LED_PIN);
    /* when power button is pressed timer is restarted */
    xGPSLEDTimer = xTimerCreate("GPS LED",
       /* The timer period in ticks. */
       gps_led_times[0][0],
       /* The timer will stop when expire. */
       pdFALSE,
       /* unique id */
       (void*)DISP_GPS_TIMER,
       /* Each timer calls the same callback when it expires. */
       vGPSLEDTimerCallback
    );
    
    display_que = xQueueCreate(5, sizeof(task_message));
}


/**
* @brief  Main Display Task.
* @param  None
* @retval None
*/
void vTaskDisplay(void* pvParameters)
{
    task_message msg;
           
    for(;;)
    {
        xQueueReceive(display_que, &msg, portMAX_DELAY);
        switch (msg.msg_opcode)
        {
            case DISP_GPS_NO_FIX:
                if (gps_led_status != GPS_LED_NO_FIX)
                {
                    GPS_LED_Start(GPS_LED_NO_FIX);
                }
                break;
            
            case DISP_GPS_FIX:
                if (gps_led_status != GPS_LED_FIX)
                {
                    GPS_LED_Start(GPS_LED_FIX);
                }               
                break;
                
            default:
                break;
        }
    }
}
