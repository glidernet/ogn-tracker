#include "usart.h"
#include <stm32l1xx.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <string.h>
#include "messages.h"
#include "cir_buf.h"

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

/* USART3 GPIO pins */
#define USART3_TX_PIN                    GPIO_Pin_10
#define USART3_TX_GPIO_PORT              GPIOC
#define USART3_TX_GPIO_CLK               RCC_AHBPeriph_GPIOC
#define USART3_TX_SOURCE                 GPIO_PinSource10
#define USART3_TX_AF                     GPIO_AF_USART3

#define USART3_RX_PIN                    GPIO_Pin_11
#define USART3_RX_GPIO_PORT              GPIOC
#define USART3_RX_GPIO_CLK               RCC_AHBPeriph_GPIOC
#define USART3_RX_SOURCE                 GPIO_PinSource11
#define USART3_RX_AF                     GPIO_AF_USART3

#define USART3_RX_BUF_SIZE               128

/* -------- variables -------- */
/* Semaphore used for USART2 TX synchronization */
static SemaphoreHandle_t xUSART2Semaphore;
/* USART2 TX data */
static uint16_t usart2_ctr;
static uint8_t* usart2_ptr;
/* USART2 RX queue */
static xQueueHandle* usart2_queue;

/* Semaphore used for USART3 TX synchronization */
static SemaphoreHandle_t xUSART3Semaphore;
/* USART3 TX data */
static uint16_t usart3_ctr;
static uint8_t* usart3_ptr;
/* USART3 RX queue */
static xQueueHandle* usart3_queue;
static uint8_t usart3_rx_buf[USART3_RX_BUF_SIZE];
static uint8_t usart3_rx_buf_pos;
static cir_buf_str* usart3_cir_buf;

/* -------- interrupt handlers -------- */
void USART2_IRQHandler(void)
{  uint8_t rs_data;
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

void USART3_IRQHandler(void)
{
   uint8_t rs_data;
   task_message msg;

   portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

   /* USART in mode Transmitter -------------------------------------------------*/
   if (USART_GetITStatus(USART3, USART_IT_TXE) == SET)
   {
      if (usart3_ctr == 0)
      {
         USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
         xSemaphoreGiveFromISR(xUSART3Semaphore, &xHigherPriorityTaskWoken);
      }
      else
      {
         USART_SendData(USART3, *usart3_ptr++);
         usart3_ctr--;
      }
   }
   
   /* USART in mode Receiver --------------------------------------------------*/
   if(USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
   {
      rs_data = USART_ReceiveData(USART3);
      if (rs_data == '$') usart3_rx_buf_pos = 0;

      usart3_rx_buf[usart3_rx_buf_pos++] = rs_data;
      if (rs_data == '\n')
      {
         if (usart3_rx_buf[0] == '$')
         {
            /* Found valid NMEA sequence */
            if (usart3_queue)
            {
               /* End sequence */
               usart3_rx_buf[usart3_rx_buf_pos] = '\0';

               msg.msg_data = (uint32_t)cir_put_data(usart3_cir_buf, usart3_rx_buf, usart3_rx_buf_pos+1);
               msg.msg_len  = usart3_rx_buf_pos;
               msg.src_id   = GPS_USART_SRC_ID;
               xQueueSendFromISR(*usart3_queue, &msg, &xHigherPriorityTaskWoken);
            }
         }
         usart3_rx_buf_pos = 0;
      }
      if (usart3_rx_buf_pos >= USART3_RX_BUF_SIZE) usart3_rx_buf_pos = 0;
   }
   portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* -------- functions -------- */
/**
* @brief  Configures the USART2 Peripheral.
* @param  USART speed
* @retval None
*/
void USART2_Config(uint32_t speed)
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
   - BaudRate = 4800 baud
   - Word Length = 8 Bits
   - one Stop Bit
   - No parity
   - Hardware flow control disabled (RTS and CTS signals)
   - Receive and transmit enabled
   */
   USART_InitStructure.USART_BaudRate   = speed;
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

/**
* @brief  Sends data over USART3 Peripheral.
* @brief  Function triggers transfer and exit (if USART clean).
* @param  data, length
* @retval None
*/
void USART3_Send(uint8_t* data, uint16_t len)
{
   /* Take access to USART3 TX  - will block if already used */
   xSemaphoreTake(xUSART3Semaphore, portMAX_DELAY);
   usart3_ptr = data; usart3_ctr = len;
   USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
}

/**
* @brief  Waits until USART3 transmission finishes.
* @param  None
* @retval None
*/
void USART3_Wait()
{
   /* Take access to USART3 TX  - will block if already used */
   xSemaphoreTake(xUSART3Semaphore, portMAX_DELAY);
   xSemaphoreGive(xUSART3Semaphore);
}

/* -------- functions -------- */
/**
* @brief  Configures the USART3 Peripheral.
* @param  USART speed
* @retval None
*/
void USART3_Config(uint32_t speed)
{
   USART_InitTypeDef USART_InitStructure;
   GPIO_InitTypeDef  GPIO_InitStructure;
   NVIC_InitTypeDef  NVIC_InitStructure;

   /* Enable GPIO clock */
   RCC_AHBPeriphClockCmd(USART3_TX_GPIO_CLK | USART3_RX_GPIO_CLK, ENABLE);

   /* Enable USART clock */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

   /* Connect PXx to USART3_Tx */
   GPIO_PinAFConfig(USART3_TX_GPIO_PORT, USART3_TX_SOURCE, USART3_TX_AF);

   /* Connect PXx to USART3_Rx */
   GPIO_PinAFConfig(USART3_RX_GPIO_PORT, USART3_RX_SOURCE, USART3_RX_AF);

   /* Configure USART Tx and Rx as alternate function push-pull */
   GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

   GPIO_InitStructure.GPIO_Pin = USART3_TX_PIN;
   GPIO_Init(USART3_TX_GPIO_PORT, &GPIO_InitStructure);

   GPIO_InitStructure.GPIO_Pin = USART3_RX_PIN;
   GPIO_Init(USART3_RX_GPIO_PORT, &GPIO_InitStructure);

   /* USART3 configuration ----------------------------------------------------*/
   /* USART3 configured as follow:
      - BaudRate = 9600 baud
      - Word Length = 8 Bits
      - one Stop Bit
      - No parity
      - Hardware flow control disabled (RTS and CTS signals)
      - Receive enabled
      */
   USART_InitStructure.USART_BaudRate   = speed;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits   = USART_StopBits_1;
   USART_InitStructure.USART_Parity     = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
   USART_Init(USART3, &USART_InitStructure);

   USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

   usart3_rx_buf_pos = 0;

   /* Enable USART3 RX IRQ */
   NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configGPS_INTERRUPT_PRIORITY;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);
   
   xUSART3Semaphore = xSemaphoreCreateMutex();
}

/**
* @brief  Sets circular buffer for USART3 received data.
* @param  None
* @retval None
*/
void USART3_SetBuf(cir_buf_str* handle)
{
   usart3_cir_buf = handle;
}

/**
* @brief  Sets queue for USART3 received data.
* @param  None
* @retval None
*/
void USART3_SetQue(xQueueHandle* handle)
{
   usart3_queue = handle;
}
