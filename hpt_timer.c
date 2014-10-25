#include <stm32l1xx.h>
#include <FreeRTOS.h>
#include <FreeRTOS_CLI.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <timers.h>

#include "hpt_timer.h"

#include "console.h"
#include "control.h"
#include "messages.h"
#include "spirit1.h"

// #define HPT_DEBUG

/* -------- defines -------- */
#define HPT_TIMER_ID  1
/* -------- variables -------- */
TimerHandle_t xHPTimer;
uint8_t       event_awaited;
HPT_Event*    current_hpt_table;

/* -------- interrupt handlers -------- */
/* -------- functions -------- */

// #ifdef HPT_DEBUG
// extern void Console_Send(const char* str, char block);
// #endif

/**
* @brief  Restart the current HPT table.
* @param  None
* @retval None
*/
void HPT_Restart(void)
{
    if (current_hpt_table)
    {
       xTimerChangePeriod(xHPTimer, current_hpt_table[0].time, 0);
       /* mark information about event number that is awaited*/
       event_awaited = 0;
       /* start the timer */
       xTimerStart(xHPTimer, 0);
    }
}

/**
* @brief  Restart the current HPT table called from ISR.
* @param  None
* @retval None
*/
BaseType_t HPT_RestartFromISR(void)
{
    BaseType_t xHigherPriorityTaskWokenCP = pdFALSE;
    BaseType_t xHigherPriorityTaskWokenR  = pdFALSE;
    
    if (current_hpt_table)
    {
       xTimerChangePeriodFromISR(xHPTimer, current_hpt_table[0].time, &xHigherPriorityTaskWokenCP);
       /* mark information about event number that is awaited*/
       event_awaited = 0;
       /* start the timer */
       xTimerStartFromISR(xHPTimer, &xHigherPriorityTaskWokenR);
    }
    if ((xHigherPriorityTaskWokenCP!=pdFALSE) || (xHigherPriorityTaskWokenR!=pdFALSE))
        return pdTRUE;
    else
        return pdFALSE;
    
}


/**
* @brief  High Precision Timer Callback.
* @param  Timer handle 
* @retval None
*/
void vHPTimerCallback(TimerHandle_t pxTimer)
{
    uint8_t new_event_scheduled = 0;
    xQueueHandle* dest_que;
    task_message  ctrl_msg;
    uint32_t      data1;
    
    data1 = current_hpt_table[event_awaited].data1;
    
    switch (current_hpt_table[event_awaited].opcode)
    {
        case HPT_END:
            xTimerStop(xHPTimer, 0);
            new_event_scheduled = 1;
            break;
            
        case HPT_RESTART:
            #ifdef HPT_DEBUG
            Console_Send("--HPT_RESTART--\r\n",0);
            #endif
            HPT_Restart();
            new_event_scheduled = 1;
            break;
            
        case HPT_GPIO_UP:
            GPIO_SetBits(GPIOC, GPIO_Pin_5);
            #ifdef HPT_DEBUG
            Console_Send("HPT_GPIO_UP\r\n",0);
            #endif
            break;
            
        case HPT_GPIO_DOWN:
            GPIO_ResetBits(GPIOC, GPIO_Pin_5);
            #ifdef HPT_DEBUG
            Console_Send("HPT_GPIO_DOWN\r\n",0);
            #endif
            break;
            
        case HPT_PREPARE_PKT:            
            #ifdef HPT_DEBUG
            Console_Send("HPT_PREPARE_PKT\r\n",0);
            #endif
            dest_que = Get_ControlQue();
            ctrl_msg.msg_data   = 0;
            ctrl_msg.msg_len    = 0;
            ctrl_msg.msg_opcode = HPT_PREPARE_PKT;
            ctrl_msg.src_id     = 0;
            xQueueSend(*dest_que, &ctrl_msg, portMAX_DELAY);
            break;
            
        case HPT_SEND_PKT:            
            #ifdef HPT_DEBUG
            Console_Send("HPT_SEND_PKT\r\n",0);
            #endif
            dest_que = Get_ControlQue();
            ctrl_msg.msg_data   = 0;
            ctrl_msg.msg_len    = 0;
            ctrl_msg.msg_opcode = HPT_SEND_PKT;
            ctrl_msg.src_id     = 0;
            xQueueSend(*dest_que, &ctrl_msg, portMAX_DELAY);
            break;
            
        case HPT_SP1_CHANNEL:            
            #ifdef HPT_DEBUG
            Console_Send("HPT_SP1_CHANNEL\r\n",0);
            #endif
            dest_que = Get_SP1Que();
            ctrl_msg.msg_data   = data1;
            ctrl_msg.msg_len    = 0;
            ctrl_msg.msg_opcode = SP1_CHG_CHANNEL;
            ctrl_msg.src_id     = 0;
            xQueueSend(*dest_que, &ctrl_msg, portMAX_DELAY);
            break;
        default:
            break;
    }
    /* In case of events not affecting hpt table timing next event from hpt table should be found */
    if (!new_event_scheduled)
    {
        /* find current time from table */
        uint32_t curr_time = current_hpt_table[event_awaited].time;
        /* get next event from table */
        event_awaited++;
        /* calculate amount of time to wait until next event */
        xTimerChangePeriod(xHPTimer, current_hpt_table[event_awaited].time - curr_time, 0);
        /* start the timer */
        xTimerStart(xHPTimer, 0);
        /* that's all, we'll see again in next vHPTimerCallback call */
    }
}

/**
* @brief  Starts the High Precision Timer.
* @param  HPT Events table
* @retval None
*/
void HPT_Start(HPT_Event* hpt_table)
{
   current_hpt_table = hpt_table;
   /* set timer expiration to first event in table */
   xTimerChangePeriod(xHPTimer, hpt_table[0].time, 0);
   /* mark information about event number that is awaited*/
   event_awaited = 0;
   /* start the timer */
   xTimerStart(xHPTimer, 0);
}

/**
* @brief  Configures the High Precision Timer.
* @param  None
* @retval None
*/
void HPT_Config(void)
{
   /* Test GPIO PC5 */
   GPIO_InitTypeDef GPIO_InitStructure;

   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

   GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
   GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
   GPIO_Init(GPIOC, &GPIO_InitStructure);
   
   GPIO_ResetBits(GPIOC, GPIO_Pin_5);
   
   xHPTimer = xTimerCreate(
      "HPTimer",
      /* The timer period in ticks. */
      HPT_MS(1000),
      /* The timer will stop when expire. */
      pdFALSE,
      /* unique id */
      ( void * )HPT_TIMER_ID,
      /* Each timer calls the same callback when it expires. */
      vHPTimerCallback
    );
}
