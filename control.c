#include "control.h"
#include <stm32l1xx.h>
#include <FreeRTOS.h>
#include <FreeRTOS_CLI.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <timers.h>
#include "messages.h"
#include "hpt_timer.h"
#include "ogn_lib.h"
#include "spirit1.h"
#include "options.h"
#include "console.h"
#include "gps.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* -------- defines -------- */
#define MAX_HPT_TABLE_LEN  16
/* -------- variables -------- */
HPT_Event hpt_table[MAX_HPT_TABLE_LEN];
TimerHandle_t xPowerDownTimer;

/* pointer to TX packet data */
uint8_t* TX_pkt_data = NULL;

/* Console task queue */
xQueueHandle  control_queue;
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

/* interrupt for falling Wakeup line */
void EXTI15_10_IRQHandler(void)
{
   BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      
   if(EXTI_GetITStatus(EXTI_Line13) != RESET)
   {
      /* Clear the EXTI line 13 pending bit */
      EXTI_ClearITPendingBit(EXTI_Line13);  
      xTimerStartFromISR(xPowerDownTimer, &xHigherPriorityTaskWoken);     
   }
   portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
   
}

/* -------- functions -------- */
xQueueHandle* Get_ControlQueue()
{
   return &control_queue;
}

/**
* @brief  Initiates tracker shut down.
* @param  None
* @retval None
*/
void PreShutDownSequence(void)
{
    Console_Send("Shutdown...\r\n", 1);
    /* Disable SP1 by SHDN line */
    Spirit1EnterShutdown();
    /* Disable GPS using NMEA sentence */
    GPS_Off();
    /* Deactivate port C GPIO lines - entering shutdown generates glitches if active */
    GPIO_DeInit(GPIOC);
    
    /* There is no way to shut-down Independent Watchdog other than CPU reset */
    /* Shut-down will be finished after reset in main()/HandlePowerUpMode() function. */
    /* So we need to configure power-up mode so after reset CPU will shut itself down. */
    
    RTC_WriteBackupRegister(SHDN_REG_NUM, SHDN_MAGIC_NUM);
    vTaskDelay(500);
    NVIC_SystemReset();  
}

void vPwrDownTimerCallback(TimerHandle_t pxTimer)
{
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) == Bit_SET)
    {
        PreShutDownSequence();
    }
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
   
   hpt_table_arr[pos].time    = HPT_MS(700);
   hpt_table_arr[pos].opcode  = HPT_SP1_CHANNEL;
   hpt_table_arr[pos].data1   = 2;  
   pos++;
   
   hpt_table_arr[pos].time    = HPT_MS(900);
   hpt_table_arr[pos].opcode  = HPT_SEND_PKT;
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
uint8_t Create_HPT_Table_Idle(HPT_Event* hpt_table_arr)
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
   NVIC_InitTypeDef NVIC_InitStructure;

   /* Power button should be pressed for 1 second to power off, xPowerDownTimer is used for counting this time */
   /* when power button is pressed timer is restarted */
   xPowerDownTimer = xTimerCreate(
      "PDTimer",
      /* The timer period in ticks. */
      1000,
      /* The timer will stop when expire. */
      pdFALSE,
      /* unique id */
      ( void * )PWR_DOWN_TIMER_ID,
      /* Each timer calls the same callback when it expires. */
      vPwrDownTimerCallback
    );
    
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
   
   /* Configure PC13 Pin (Wakeup) as GPIO interrupt */  
   GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
   GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
   GPIO_Init(GPIOC, &GPIO_InitStructure);

   SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13); 

   EXTI_InitStructure.EXTI_Line    = EXTI_Line13;
   EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);
   
   /* enable Power Button input lines interrupt */
   NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configPWR_BTN_INTERRUPT_PRIORITY;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
  
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
        case MODE_IDLE:          
            break;
            
        case MODE_CW:
            /* wait for Spirit1 task */
            vTaskDelay(1000);
            sp1_task_queue = Get_SP1Queue();
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
            
         case MODE_RX:
            /* wait for Spirit1 task */
            vTaskDelay(1000);
            sp1_task_queue = Get_SP1Queue();
            if (sp1_task_queue)
            {
                /* Start RX */
                sp1_msg.msg_data   = 0;
                sp1_msg.msg_len    = 0;
                sp1_msg.msg_opcode = SP1_START_RX;
                sp1_msg.src_id     = CONTROL_SRC_ID;
                xQueueSend(*sp1_task_queue, &sp1_msg, portMAX_DELAY);
            }
            break;
        
        default:
            /* do nothing for other modes */
            break;
    }
}

/* borrowed from commands.c */
static uint8_t print_hex_val(uint8_t data, char* dest)
{
   char t;
   t = data>>4;
   dest[0] = t > 9 ? t+'A'-0x0A : t+'0';
   t = data&0x0F;
   dest[1] = t > 9 ? t+'A'-0x0A : t+'0';
   return 2;
}


void Print_packet(rcv_packet_str* packet)
{
    char buffer[80];
    int i, Neg=0, ctr=0; 
    
    float rssi = packet->rssi;
    int lqi = packet->lqi;
    int pqi = packet->pqi;
    int sqi = packet->sqi;
    
    if(rssi<0) { Neg=1; rssi=(-rssi); }
    int Int = (int)floor(rssi);
    int Frac = (int)floor((rssi-Int)*10);
    if(Neg) Int=(-Int);
    sprintf(buffer, "Packet received: RSSI: %+d.%ddBm, LQI: %d, PQI: %d, SQI: %d\r\n", Int, Frac, lqi, pqi, sqi);
    Console_Send(buffer, 1);
    
    for (i=0; i < OGN_PKT_LEN; i++)
    {
       ctr+= print_hex_val(packet->packet_data_ptr[i], &buffer[ctr]);
    }
    buffer[ctr++] = '\r'; buffer[ctr++] = '\n';
    buffer[ctr++] = '\0';
    
    Console_Send(buffer, 1);
    
}

/**
* @brief  Handle messages received from Spirit1 task.
* @param  Message structure.
* @retval None
*/
static void Handle_sp1_msgs(task_message* msg)
{
    switch (msg->msg_opcode)
    {
        case SP1_OUT_PKT_READY:
            Print_packet((rcv_packet_str*)msg->msg_data);
            break;
            
        default:
            break;
    }
}

/**
* @brief  Handle messages received from High Precision Timer task.
* @param  Message structure.
* @retval None
*/
static void Handle_hpt_msgs(task_message* msg)
{
    task_message sp1_msg;
    
    switch (msg->msg_opcode)
    {
        case HPT_PREPARE_PKT:
            TX_pkt_data = OGN_PreparePacket();
            break;

        case HPT_SEND_PKT:
            if (TX_pkt_data)
            {
                sp1_msg.msg_data   = (uint32_t)TX_pkt_data;
                sp1_msg.msg_len    = OGN_PKT_LEN;
                sp1_msg.msg_opcode = SP1_SEND_OGN_PKT;
                sp1_msg.src_id     = CONTROL_SRC_ID;
                xQueueHandle* sp1_task_queue = Get_SP1Queue();
                xQueueSend(*sp1_task_queue, &sp1_msg, portMAX_DELAY);
            }
            break;
            
        default:
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
   task_message msg;
   oper_modes oper_mode;
   
   control_queue = xQueueCreate(10, sizeof(task_message));
      
   /* Select timer table depending on operation mode */
   oper_mode = *(uint8_t *)GetOption(OPT_OPER_MODE); 
   switch (oper_mode)
   {
        case MODE_OGN:      
            Create_HPT_Table_OGN(hpt_table);
            break;
        case MODE_IDLE:
        case MODE_CW: 
        case MODE_RX:          
            Create_HPT_Table_Idle(hpt_table);
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
        xQueueReceive(control_queue, &msg, portMAX_DELAY);
        switch (msg.src_id)
        {
            case HPT_SRC_ID:
                Handle_hpt_msgs(&msg);
                break;
            
            case SPIRIT1_SRC_ID:
                Handle_sp1_msgs(&msg);
                break;
                
            default:
                break;
      }
   }
}
