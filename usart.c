#include "usart.h"
#include <stm32l1xx.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <string.h>
#include "messages.h"

/* -------- defines -------- */

/* USART2 GPIO pins */
#define USART2_TX_PIN                    GPIO_Pin_2
#define USART2_TX_GPIO_PORT              GPIOA
#define USART2_TX_GPIO_CLK               RCC_AHBPeriph_GPIOA
#define USART2_TX_SOURCE                 GPIO_PinSource2
#define USART2_TX_AF                     GPIO_AF_USART2

#define USART2_RX_PIN                    GPIO_Pin_3
#define USART2_RX_GPIO_PORT              GPIOA
#define USART2_RX_GPIO_CLK               RCC_AHBPeriph_GPIOA
#define USART2_RX_SOURCE                 GPIO_PinSource3
#define USART2_RX_AF                     GPIO_AF_USART2

/* -------- variables -------- */
/* Semaphore used for USART2 TX synchronization */
static SemaphoreHandle_t xUSART2Semaphore;
static uint16_t usart2_ctr;
static uint8_t* usart2_ptr;
static xQueueHandle* usart2_queue;

/* -------- interrupt handlers -------- */
void USART2_IRQHandler(void)
{
   uint8_t rs_data;
   task_message msg;

   portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

   /* USART in mode Transmitter -------------------------------------------------*/
   if (USART_GetITStatus(USART2, USART_IT_TXE) == SET)
   {
      if (usart2_ctr == 0)
      {
         USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
         xSemaphoreGiveFromISR(xUSART2Semaphore, &xHigherPriorityTaskWoken);
      }
      else
      {
         USART_SendData(USART2, *usart2_ptr++);
         usart2_ctr--;
      }
   }

   /* USART in mode Receiver --------------------------------------------------*/
   if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
   {
      rs_data = USART_ReceiveData(USART2);
      if (usart2_queue)
      {
         msg.src_id     = CONSOLE_USART_SRC_ID;
         msg.msg_opcode = rs_data;
         xQueueSendFromISR(*usart2_queue, &msg, &xHigherPriorityTaskWoken);
      }
   }
   portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* -------- functions -------- */
/**
* @brief  Configures the USART2 Peripheral.
* @param  None
* @retval None
*/
void USART2_Config(void)
{
   USART_InitTypeDef USART_InitStructure;
   GPIO_InitTypeDef  GPIO_InitStructure;
   NVIC_InitTypeDef  NVIC_InitStructure;
   
   /* Enable GPIO clock */
   RCC_AHBPeriphClockCmd(USART2_TX_GPIO_CLK | USART2_RX_GPIO_CLK, ENABLE);
  
   /* Enable USART clock */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  
   /* Connect PXx to USART2_Tx */
   GPIO_PinAFConfig(USART2_TX_GPIO_PORT, USART2_TX_SOURCE, USART2_TX_AF);
  
   /* Connect PXx to USART2_Rx */
   GPIO_PinAFConfig(USART2_RX_GPIO_PORT, USART2_RX_SOURCE, USART2_RX_AF);
  
   /* Configure USART Tx and Rx as alternate function push-pull */
   GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

   GPIO_InitStructure.GPIO_Pin = USART2_TX_PIN;
   GPIO_Init(USART2_TX_GPIO_PORT, &GPIO_InitStructure);

   GPIO_InitStructure.GPIO_Pin = USART2_RX_PIN;
   GPIO_Init(USART2_RX_GPIO_PORT, &GPIO_InitStructure);
  
   /* USART2 configuration ----------------------------------------------------*/
   /* USART2 configured as follow:
   - BaudRate = 9600 baud  
   - Word Length = 8 Bits
   - one Stop Bit
   - No parity
   - Hardware flow control disabled (RTS and CTS signals)
   - Receive and transmit enabled
   */
   USART_InitStructure.USART_BaudRate   = 9600;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits   = USART_StopBits_1;
   USART_InitStructure.USART_Parity     = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
   USART_Init(USART2, &USART_InitStructure);

   USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

   /* Enable USART2 RX IRQ */
   NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configCONSOLE_INTERRUPT_PRIORITY;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

   xUSART2Semaphore = xSemaphoreCreateMutex();
}

/**
* @brief  Sets queue for USART2 received data.
* @param  None
* @retval None
*/
void USART2_SetQue(xQueueHandle* handle)
{
   usart2_queue = handle;
}

/**
* @brief  Sends data over USART2 Peripheral.
* @brief  Function triggers transfer and exit (if USART clean).
* @param  data, length
* @retval None
*/
void USART2_Send(uint8_t* data, uint16_t len)
{
   /* Take access to USART2 TX  - will block if already used */
   xSemaphoreTake(xUSART2Semaphore, portMAX_DELAY);
   usart2_ptr = data; usart2_ctr = len;
   USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}

/**
* @brief  Waits until USART2 transmission finishes.
* @param  None
* @retval None
*/
void USART2_Wait()
{
   /* Take access to USART2 TX  - will block if already used */
   xSemaphoreTake(xUSART2Semaphore, portMAX_DELAY);
   xSemaphoreGive(xUSART2Semaphore);
}
