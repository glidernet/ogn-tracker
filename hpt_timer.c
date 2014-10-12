#include "hpt_timer.h"
#include <stm32l1xx.h>
#include <FreeRTOS.h>
#include <FreeRTOS_CLI.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <timers.h>

/* -------- defines -------- */
#define HPT_TIMER_ID  1
/* -------- variables -------- */
TimerHandle_t xHPTimer;
/* -------- interrupt handlers -------- */
/* -------- functions -------- */

/**
* @brief  High Precision Timer Callback.
* @param  Timer handle 
* @retval None
*/
void vHPTimerCallback(TimerHandle_t pxTimer)
{
    
}

/**
* @brief  Starts the High Precision Timer.
* @param  None
* @retval None
*/
void HPT_Start(void)
{
   xTimerStart(xHPTimer, 0);
}

/**
* @brief  Configures the High Precision Timer.
* @param  None
* @retval None
*/
void HPT_Config(void)
{
   xHPTimer = xTimerCreate(
      "HPTimer",
      /* The timer period in ticks. */
      HPT_MS(1000),
      /* The timer will auto-reload when expire. */
      pdTRUE,
      /* unique id */
      ( void * )HPT_TIMER_ID,
      /* Each timer calls the same callback when it expires. */
      vHPTimerCallback
    );
}
