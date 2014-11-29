#include "control.h"
#include <stm32l1xx.h>
#include <FreeRTOS.h>
#include <FreeRTOS_CLI.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include "messages.h"
#include "hpt_timer.h"
#include "ogn_lib.h"
#include "spirit1.h"
#include "options.h"

/* -------- defines -------- */
#define MAX_HPT_TABLE_LEN  16
/* -------- variables -------- */
HPT_Event hpt_table[MAX_HPT_TABLE_LEN];

/* Console task queue */
xQueueHandle  control_que;
/* -------- interrupt handlers -------- */

/* interrupt for raising GPS_PPS line */
void EXTI9_5_IRQHandler(void)
{
   BaseType_t xHigherPriorityTaskWoken = pdFALSE;
   
   if(EXTI_GetITStatus(EXTI_Line6) != RESET)
   {
      /* Clear the EXTI line 6 pending bit */
      EXTI_ClearITPendingBit(EXTI_Line6); 
      xHigherPriorityTaskWoken = HPT_RestartFromISR();      
   }
   portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
   
}
/* -------- functions -------- */
xQueueHandle* Get_ControlQue()
{
   return &control_que;
}

/**
* @brief  Configures the High Precision Timer Table for OGN oper. mode.
* @param  pointer to hpt_table data to be filled.
* @retval length of filled data.
*/
uint8_t Create_HPT_Table_OGN(HPT_Event* hpt_table_arr)
{
   uint8_t pos = 0;
   
   hpt_table_arr[pos].time    = HPT_MS(200);
   hpt_table_arr[pos].opcode  = HPT_SP1_CHANNEL;
   hpt_table_arr[pos].data1   = 4;  
   pos++;
   
   hpt_table_arr[pos].time    = HPT_MS(500);
   hpt_table_arr[pos].opcode  = HPT_SEND_PKT;
   pos++;
   
   hpt_table_arr[pos].time    = HPT_MS(500);
   hpt_table_arr[pos].opcode  = HPT_GPIO_UP;
   pos++;
   
   hpt_table_arr[pos].time    = HPT_MS(700);
   hpt_table_arr[pos].opcode  = HPT_SP1_CHANNEL;
   hpt_table_arr[pos].data1   = 2;  
   pos++;
   
   hpt_table_arr[pos].time    = HPT_MS(900);
   hpt_table_arr[pos].opcode  = HPT_SEND_PKT;
   pos++;
   
   hpt_table_arr[pos].time    = HPT_MS(900);
   hpt_table_arr[pos].opcode  = HPT_GPIO_DOWN;
   pos++;
   
   hpt_table_arr[pos].time    = HPT_MS(925);
   hpt_table_arr[pos].opcode  = HPT_IWDG_RELOAD;
   pos++;
   
   hpt_table_arr[pos].time    = HPT_MS(950);
   hpt_table_arr[pos].opcode  = HPT_PREPARE_PKT;
   pos++;
	
   hpt_table_arr[pos].time    = HPT_MS(1000);
   hpt_table_arr[pos].opcode  = HPT_RESTART;
   pos++;
   
   return pos;
   
}

/**
* @brief  Configures the High Precision Timer Table for CW oper. mode.
* @param  pointer to hpt_table data to be filled.
* @retval length of filled data.
*/
uint8_t Create_HPT_Table_CW(HPT_Event* hpt_table_arr)
{
   uint8_t pos = 0;
    
   hpt_table_arr[pos].time    = HPT_MS(925);
   hpt_table_arr[pos].opcode  = HPT_IWDG_RELOAD;
   pos++;
   	
   hpt_table_arr[pos].time    = HPT_MS(1000);
   hpt_table_arr[pos].opcode  = HPT_RESTART;
   pos++;
   
   return pos;
   
}

/**
* @brief  Configures Independent Watchdog IWDG.
* @param  None
* @retval None
*/
void IWDG_Config(void)
{
    uint8_t iwdg_dis = *(uint8_t *)GetOption(OPT_IWDG);
    
    if (!iwdg_dis)
    {    
        /* Enable write access to IWDG_PR and IWDG_RLR registers */
        IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
        /* IWDG Clock: 40kHz/64 */
        IWDG_SetPrescaler(IWDG_Prescaler_64);
        /* Configure the IWDG counter value - 6 second at /64 prescaler*/
        IWDG_SetReload(0xfff);
        /* Reload Counter */
        IWDG_ReloadCounter();
        /* Enable IWDG */
        IWDG_Enable();
    }
}
/**
* @brief  Configures the Control Task Peripherals.
* @param  None
* @retval None
*/
void Control_Config(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
   EXTI_InitTypeDef EXTI_InitStructure;

   /* Configure PC6 Pin (GPS_PPS) as GPIO interrupt */
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);  
   /* Enable SYSCFG clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
   
   GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
   GPIO_Init(GPIOC, &GPIO_InitStructure);

   SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource6); 

   EXTI_InitStructure.EXTI_Line    = EXTI_Line6;
   EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);
    
   IWDG_Config();
   HPT_Config();

}

/**
* @brief  Executes necessary action before entering event loop.
* @param  mode - selected mode.
* @retval None
*/
void StartMode(oper_modes mode)
{
    task_message sp1_msg;
    xQueueHandle* sp1_task_queue;
    
    switch(mode)
    {
        case MODE_CW:
            /* wait for Spirit1 task */
            vTaskDelay(1000);
            sp1_task_queue = Get_SP1Que();
            if (sp1_task_queue)
            {
                /* Start CW */
                sp1_msg.msg_data   = 0;
                sp1_msg.msg_len    = 0;
                sp1_msg.msg_opcode = SP1_START_CW;
                sp1_msg.src_id     = CONTROL_SRC_ID;
                xQueueSend(*sp1_task_queue, &sp1_msg, portMAX_DELAY);
            }
            break;
        
        default:
            /* do nothing for other modes */
            break;
    }
}

/**
* @brief  Main Control Task.
* @param  None
* @retval None
*/
void vTaskControl(void* pvParameters)
{
   NVIC_InitTypeDef NVIC_InitStructure;
   task_message msg, sp1_msg;
   uint8_t* pkt_data = NULL;
   oper_modes oper_mode;
   
   control_que = xQueueCreate(10, sizeof(task_message));
      
   /* Select timer table depending on operation mode */
   oper_mode = *(uint8_t *)GetOption(OPT_OPER_MODE); 
   switch (oper_mode)
   {
        case MODE_OGN:      
            Create_HPT_Table_OGN(hpt_table);
            break;
        case MODE_CW:      
            Create_HPT_Table_CW(hpt_table);
            break;
        default:
            /* in case of error - fall to OGN mode */         
            Create_HPT_Table_OGN(hpt_table);
            break;
   } 
   HPT_Start(hpt_table);

   /* enable GPS PPS input lines interrupts */
   NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configGPS_PPS_INTERRUPT_PRIORITY;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
   
   StartMode(oper_mode);
   
   for(;;)
   {
      xQueueReceive(control_que, &msg, portMAX_DELAY);
      switch (msg.msg_opcode)
      {
         case HPT_PREPARE_PKT:
            pkt_data = OGN_PreparePacket();
            break;

         case HPT_SEND_PKT:
            if (pkt_data)
            {
                sp1_msg.msg_data   = (uint32_t)pkt_data;
                sp1_msg.msg_len    = OGN_PKT_LEN;
                sp1_msg.msg_opcode = SP1_SEND_OGN_PKT;
                sp1_msg.src_id     = CONTROL_SRC_ID;
                xQueueHandle* sp1_task_queue = Get_SP1Que();
                xQueueSend(*sp1_task_queue, &sp1_msg, portMAX_DELAY);
            }
            break;
         default:
            break;
      }
   }
}
