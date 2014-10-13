#include "control.h"
#include <stm32l1xx.h>
#include <FreeRTOS.h>
#include <FreeRTOS_CLI.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include "messages.h"
#include "hpt_timer.h"

/* -------- defines -------- */
#define MAX_HPT_TABLE_LEN  8
/* -------- variables -------- */
HPT_Event hpt_table[MAX_HPT_TABLE_LEN];

/* Console task queue */
xQueueHandle  control_que;
/* -------- interrupt handlers -------- */
/* -------- functions -------- */

/**
* @brief  Configures the High Precision Timer Table.
* @param  pointer to hpt_table data to be filled.
* @retval length of filled data.
*/
uint8_t Create_HPT_Table(HPT_Event* hpt_table_arr)
{
   uint8_t pos = 0;
   hpt_table_arr[pos].time    = HPT_MS(150);
   hpt_table_arr[pos].opcode  = HPT_GPIO_UP;
   pos++;
   
   hpt_table_arr[pos].time    = HPT_MS(151);
   hpt_table_arr[pos].opcode  = HPT_GPIO_DOWN;
   pos++;
   
   hpt_table_arr[pos].time    = HPT_MS(152);
   hpt_table_arr[pos].opcode  = HPT_GPIO_UP;
   pos++;
   
   hpt_table_arr[pos].time    = HPT_MS(153);
   hpt_table_arr[pos].opcode  = HPT_GPIO_DOWN;
   pos++;
   
	
   hpt_table_arr[pos].time    = HPT_MS(1000);
   hpt_table_arr[pos].opcode  = HPT_RESTART;
   pos++;
   
   return pos;
   
}

/**
* @brief  Configures the Control Task Peripherals.
* @param  None
* @retval None
*/
void Control_Config(void)
{
   HPT_Config();
}

/**
* @brief  Main Control Task.
* @param  None
* @retval None
*/
void vTaskControl(void* pvParameters)
{  
   task_message msg;
   
   control_que = xQueueCreate(10, sizeof(task_message));
   Create_HPT_Table(hpt_table);
   HPT_Start(hpt_table);
   
   for(;;)
   {
      xQueueReceive(control_que, &msg, portMAX_DELAY);
   }
}
