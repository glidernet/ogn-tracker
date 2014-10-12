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
/* -------- variables -------- */
/* Console task queue */
xQueueHandle  control_que;
/* -------- interrupt handlers -------- */
/* -------- functions -------- */
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
   HPT_Start();
   
   for(;;)
   {
      xQueueReceive(control_que, &msg, portMAX_DELAY);
   }
}
