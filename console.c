#include "console.h"
#include <stm32l1xx.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <string.h>
#include "messages.h"
#include "usart.h"

/* -------- defines -------- */
/* -------- variables -------- */
/* Console task queue */
xQueueHandle  console_que;
static const char * const pcWelcomeMessage = "\r\nOGN Tracker Console.\r\n";

/* -------- interrupt handlers -------- */
/* -------- functions -------- */
/**
* @brief  Configures the Console Task Peripherals.
* @param  None
* @retval None
*/
void Console_Config(void)
{
   USART2_Config();
}

/**
* @brief  Sends Console string.
* @param  None
* @retval None
*/
void Console_Send(const char* str)
{
   USART2_Send((uint8_t*)str, strlen(str));
}

/**
* @brief  Sends Console char.
* @param  None
* @retval None
*/
void Console_Send_Char(char ch)
{
   static uint8_t data;
   data = ch;
   USART2_Send(&data, 1);
}



/**
* @brief  Main Console Task.
* @param  None
* @retval None
*/
void vTaskConsole(void* pvParameters)
{
   task_message msg;
   uint8_t opcode;

   console_que = xQueueCreate(10, sizeof(task_message));
   USART2_SetQue(&console_que);

   /* Enable USART2 */
   USART_Cmd(USART2, ENABLE);

   Console_Send(pcWelcomeMessage);

   for(;;)
   {
      xQueueReceive(console_que, &msg, portMAX_DELAY);
      switch (msg.src_id)
      {
          case CONSOLE_USART_SRC_ID:
          {
             opcode = msg.msg_opcode;
             Console_Send("Received: ");
             Console_Send_Char(opcode);
             Console_Send("\r\n");
             break;
          }
          default:
          {
             break;
          }
      }
   }
}
