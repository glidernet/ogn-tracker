#include "hpt_timer.h"
#include <stdio.h>
#include <stm32l1xx.h>
#include <FreeRTOS.h>
#include <FreeRTOS_CLI.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <timers.h>
#include "console.h"
#include "control.h"
#include "messages.h"
#include "spirit1.h"
#include "timer_const.h"

/* -------- defines -------- */
/* -------- variables -------- */
static TimerHandle_t xHPTimer;
static uint8_t       event_awaited;
static HPT_Event*    current_hpt_table;
static char          log_buf[40];
static char          debug_enabled = 0;
static char          pps_synced = 0;

const char* const hpt_opcodes_str[] = {
    "**RSRT**",
    "GPIO_UP ",    
    "GPIO_DWN",   
    "PREP_PKT",  
    "COPY_PKT",     
    "SP1_CHAN", 
    "TX_PKT  ",      
    "TX_LBT  ",   
    "IWDG_RLD"   
};

/* -------- interrupt handlers -------- */
/* -------- functions -------- */
/**
* @brief  Control HPT logging.
* @param  0 - disabled, any other - enabled
* @retval None
*/
void HPT_Debug(uint8_t state)
{
    debug_enabled = state;
}

/**
* @brief  Restart the current HPT table called from ISR.
* @param  None
* @retval None
*/
BaseType_t HPT_RestartFromISR(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if (!pps_synced && current_hpt_table)
    {
       xTimerChangePeriodFromISR(xHPTimer, current_hpt_table[0].time, &xHigherPriorityTaskWoken);
       /* mark information about event number that is awaited*/
       event_awaited = 0;
       pps_synced = 1;
       Console_Send("HPTimer synced to PPS\r\n", 0);
    }
    return xHigherPriorityTaskWoken;
}


/**
* @brief  High Precision Timer Callback.
* @param  Timer handle 
* @retval None
*/
void vHPTimerCallback(TimerHandle_t pxTimer)
{
    xQueueHandle* dest_queue;
    task_message  ctrl_msg;
    uint32_t      curr_event_idx, curr_event_data1, current_time, wait_time;
    hpt_opcodes   curr_event_opcode;
       
    curr_event_idx    = event_awaited;
    curr_event_opcode = current_hpt_table[curr_event_idx].opcode;
    curr_event_data1  = current_hpt_table[curr_event_idx].data1;
   
    if (curr_event_opcode == HPT_RESTART)
    {
        current_time = 0; 
        /* get first event from table */
        event_awaited = 0;
    }
    else
    {
        current_time = current_hpt_table[event_awaited].time;
        /* get next event from table */
        event_awaited++;
    }
    /* calculate number of ticks to wait until next event */   
    wait_time = current_hpt_table[event_awaited].time - current_time;
   
    xTimerChangePeriod(xHPTimer, wait_time, 0);
    /* start the timer */
    xTimerStart(xHPTimer, 0);

    /* perform event action */
    switch (curr_event_opcode)
    {       
        case HPT_RESTART:
            break;
            
        case HPT_GPIO_UP:
            //GPIO_SetBits(...);
            break;
            
        case HPT_GPIO_DOWN:
            //GPIO_ResetBits(...);
            break;
            
        case HPT_PREPARE_PKT:            
            dest_queue = Get_ControlQueue();
            ctrl_msg.msg_data   = 0;
            ctrl_msg.msg_len    = 0;
            ctrl_msg.msg_opcode = HPT_PREPARE_PKT;
            ctrl_msg.src_id     = HPT_SRC_ID;
            xQueueSend(*dest_queue, &ctrl_msg, portMAX_DELAY);
            break;
            
        case HPT_COPY_PKT:            
            dest_queue = Get_ControlQueue();
            ctrl_msg.msg_data   = 0;
            ctrl_msg.msg_len    = 0;
            ctrl_msg.msg_opcode = HPT_COPY_PKT;
            ctrl_msg.src_id     = HPT_SRC_ID;
            xQueueSend(*dest_queue, &ctrl_msg, portMAX_DELAY);
            break;
            
        case HPT_SP1_CHANNEL:            
            dest_queue = Get_SP1Queue();
            ctrl_msg.msg_data   = curr_event_data1;
            ctrl_msg.msg_len    = 0;
            ctrl_msg.msg_opcode = SP1_CHG_CHANNEL;
            ctrl_msg.src_id     = HPT_SRC_ID;
            xQueueSend(*dest_queue, &ctrl_msg, portMAX_DELAY);
            break;
        
        case HPT_TX_PKT:            
            dest_queue = Get_SP1Queue();
            ctrl_msg.msg_data   = 0;
            ctrl_msg.msg_len    = 0;
            ctrl_msg.msg_opcode = SP1_TX_PACKET;
            ctrl_msg.src_id     = HPT_SRC_ID;
            xQueueSend(*dest_queue, &ctrl_msg, portMAX_DELAY);
            break;
            
        case HPT_TX_PKT_LBT:            
            dest_queue = Get_SP1Queue();
            ctrl_msg.msg_data   = curr_event_data1;
            ctrl_msg.msg_len    = 0;
            ctrl_msg.msg_opcode = SP1_TX_PACKET_LBT;
            ctrl_msg.src_id     = HPT_SRC_ID;
            xQueueSend(*dest_queue, &ctrl_msg, portMAX_DELAY);
            break;
            
        case HPT_IWDG_RELOAD:            
            IWDG_ReloadCounter();
            break;
            
        default:
            break;
    }
    
    if (debug_enabled)
    {
        sprintf(log_buf, "%d(%s[%4d]), (%d)->%d \r\n", 
            (int)curr_event_idx,
            hpt_opcodes_str[curr_event_opcode],
            (int)curr_event_data1,
            (int)wait_time,
            (int)event_awaited);
        Console_Send(log_buf, 1);
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
    xHPTimer = xTimerCreate(
      "HPTimer",
      /* The timer period in ms. */
      TIMER_MS(1000),
      /* The timer will stop when expire. */
      pdFALSE,
      /* unique id */
      ( void * )HPT_TIMER_ID,
      /* Each timer calls the same callback when it expires. */
      vHPTimerCallback
    );
}
