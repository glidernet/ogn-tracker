#include "spi.h"
#include <stm32l1xx.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

/* -------- defines -------- */
/* RM0038: STM32L reference manual */
/* Table 40. Summary of DMA1 requests for each channel */
#define DMA_SPI1_RX_CH        DMA1_Channel2
#define DMA_SPI1_TX_CH        DMA1_Channel3

#define SPI1_SCK_PIN          GPIO_Pin_5
#define SPI1_SCK_GPIO_PORT    GPIOA
#define SPI1_SCK_GPIO_CLK     RCC_AHBPeriph_GPIOA
#define SPI1_SCK_SOURCE       GPIO_PinSource5

#define SPI1_MISO_PIN         GPIO_Pin_6
#define SPI1_MISO_GPIO_PORT   GPIOA
#define SPI1_MISO_GPIO_CLK    RCC_AHBPeriph_GPIOA
#define SPI1_MISO_SOURCE      GPIO_PinSource6

#define SPI1_MOSI_PIN         GPIO_Pin_7
#define SPI1_MOSI_GPIO_PORT   GPIOA
#define SPI1_MOSI_GPIO_CLK    RCC_AHBPeriph_GPIOA
#define SPI1_MOSI_SOURCE      GPIO_PinSource7

#define SPI1_CE_PIN           GPIO_Pin_4
#define SPI1_CE_GPIO_PORT     GPIOA
#define SPI1_CE_GPIO_CLK      RCC_AHBPeriph_GPIOA
#define SPI1_CE_SOURCE        GPIO_PinSource4

/* -------- variables -------- */
/* Semaphore used for SPI1 transfers synchronization */
static SemaphoreHandle_t xSPI1Semaphore;

static DMA_InitTypeDef  DMA_InitTX;
static DMA_InitTypeDef  DMA_InitRX;

/* -------- interrupt handlers -------- */
/* interrupt raised after SPI1 RX transfer finish */
void DMA1_Channel2_IRQHandler(void)
{
   static signed portBASE_TYPE xHigherPriorityTaskWoken;
   xHigherPriorityTaskWoken = pdFALSE;

   /* Test on DMA1 Channel2 Transfer Complete interrupt */
   if(DMA_GetITStatus(DMA1_IT_TC2))
   {
      /* DMA1 RX finished the transfer */
      /* Clear DMA1 Channel2 Global interrupt pending bits */
      DMA_ClearITPendingBit(DMA1_IT_GL2);

      /* Disable the DMA channels */
      DMA_Cmd(DMA_SPI1_RX_CH, DISABLE);

      /* Set CE line after transfer */
      GPIO_SetBits(SPI1_CE_GPIO_PORT, SPI1_CE_PIN);

      xSemaphoreGiveFromISR(xSPI1Semaphore, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
   }
}

/* interrupt raised after SPI1 TX transfer finish */
void DMA1_Channel3_IRQHandler(void)
{
   /* Test on DMA1 Channel3 Transfer Complete interrupt */
   if(DMA_GetITStatus(DMA1_IT_TC3))
   {
      /* DMA1 TX finished the transfer */
      /* Clear DMA1 Channel3 Global interrupt pending bits */
      DMA_ClearITPendingBit(DMA1_IT_GL3);

      /* Disable the DMA channels */
      DMA_Cmd(DMA_SPI1_TX_CH, DISABLE);
   }
}

/* -------- functions -------- */
/**
* @brief  Configures the SPI1 Peripheral.
* @param  None
* @retval None
*/
void SPI1_Config(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
   SPI_InitTypeDef  SPI_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure;

   /* Enable the SPI periph */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

   /* Enable the DMA peripheral */
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

   /* Enable SCK, MOSI, MISO and CE GPIO clocks */
   RCC_AHBPeriphClockCmd(SPI1_SCK_GPIO_CLK | SPI1_MISO_GPIO_CLK | SPI1_MOSI_GPIO_CLK| SPI1_CE_GPIO_CLK, ENABLE);

   /* Enable alternate function of GPIO */
   GPIO_PinAFConfig(SPI1_SCK_GPIO_PORT,  SPI1_SCK_SOURCE,  GPIO_AF_SPI1);
   GPIO_PinAFConfig(SPI1_MOSI_GPIO_PORT, SPI1_MOSI_SOURCE, GPIO_AF_SPI1);
   GPIO_PinAFConfig(SPI1_MISO_GPIO_PORT, SPI1_MISO_SOURCE, GPIO_AF_SPI1);
   //GPIO_PinAFConfig(SPI1_CE_GPIO_PORT,   SPI1_CE_SOURCE,   GPIO_AF_SPI1);

   /* Enable NSS output for master mode */
   //SPI_SSOutputCmd(SPI1, ENABLE);

   /* --- Init common SPI pins configuration --- */
   GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

   /* SPI SCK pin configuration */
   GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN;
   GPIO_Init(SPI1_SCK_GPIO_PORT, &GPIO_InitStructure);

   /* SPI MOSI pin configuration */
   GPIO_InitStructure.GPIO_Pin = SPI1_MOSI_PIN;
   GPIO_Init(SPI1_MOSI_GPIO_PORT, &GPIO_InitStructure);

   /* SPI MISO pin configuration */
   GPIO_InitStructure.GPIO_Pin = SPI1_MISO_PIN;
   GPIO_Init(SPI1_MISO_GPIO_PORT, &GPIO_InitStructure);

   /* SPI CE pin configuration */
   GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
   GPIO_InitStructure.GPIO_Pin   = SPI1_CE_PIN;

   GPIO_Init(SPI1_CE_GPIO_PORT, &GPIO_InitStructure);

   /* Set CE line before transfer*/
   GPIO_SetBits(SPI1_CE_GPIO_PORT, SPI1_CE_PIN);

   /* --- SPI1 Configuration Master mode, MCU controlled --- */
   SPI_I2S_DeInit(SPI1);
   SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
   SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
   SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
   SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
   SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
   SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
   SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
   SPI_InitStructure.SPI_CRCPolynomial = 7;
   SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
   SPI_Init(SPI1, &SPI_InitStructure);

   /* Enable the SPI Rx and Tx DMA requests */
   SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
   SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

   /* --- Configure DMA SPI1 RX channel interrupt (transfer complete & error) --- */
   DMA_ITConfig(DMA_SPI1_RX_CH , DMA_IT_TC | DMA_IT_TE, ENABLE);

   /* --- Configure DMA SPI1 TX channel interrupt (transfer complete & error) --- */
   DMA_ITConfig(DMA_SPI1_TX_CH , DMA_IT_TC | DMA_IT_TE, ENABLE);


   /* Enable DMA1 channel2 - data received IRQ Channel */
   NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configSPI_BUS_INTERRUPT_PRIORITY;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

   /* Enable DMA1 channel3 - data transmitted IRQ Channel */
   NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configSPI_BUS_INTERRUPT_PRIORITY;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);


   /* Init. DMA SPI TX structures */
   DMA_InitTX.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
   DMA_InitTX.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
   DMA_InitTX.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
   DMA_InitTX.DMA_MemoryInc          = DMA_MemoryInc_Enable;
   DMA_InitTX.DMA_Mode               = DMA_Mode_Normal;
   DMA_InitTX.DMA_M2M                = DMA_M2M_Disable;
   DMA_InitTX.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
   DMA_InitTX.DMA_DIR                = DMA_DIR_PeripheralDST;
   DMA_InitTX.DMA_Priority           = DMA_Priority_High;
   /* will be filled before transfer */
   DMA_InitTX.DMA_BufferSize         = 0;
   DMA_InitTX.DMA_MemoryBaseAddr     = 0;

   /* Init. DMA SPI RX structures */
   DMA_InitRX.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
   DMA_InitRX.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
   DMA_InitRX.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
   DMA_InitRX.DMA_MemoryInc          = DMA_MemoryInc_Enable;
   DMA_InitRX.DMA_Mode               = DMA_Mode_Normal;
   DMA_InitRX.DMA_M2M                = DMA_M2M_Disable;
   DMA_InitRX.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
   DMA_InitRX.DMA_DIR                = DMA_DIR_PeripheralSRC;
   DMA_InitRX.DMA_Priority           = DMA_Priority_High;
   /* will be filled before transfer */
   DMA_InitRX.DMA_BufferSize         = 0;
   DMA_InitRX.DMA_MemoryBaseAddr     = 0;

   xSPI1Semaphore = xSemaphoreCreateMutex();

   /* Enable the SPI1 */
   SPI_Cmd(SPI1, ENABLE);

}

void SPI1_Send(uint8_t* data_tx, uint8_t* data_rx, uint8_t len)
{
   volatile int i;

   /* Take access to SPI1  - will block if already used */
   xSemaphoreTake(xSPI1Semaphore, portMAX_DELAY);

   DMA_InitTX.DMA_BufferSize = len;
   DMA_InitTX.DMA_MemoryBaseAddr = (uint32_t)data_tx;
   DMA_Init(DMA_SPI1_TX_CH, &DMA_InitTX);

   DMA_InitRX.DMA_BufferSize = len;
   DMA_InitRX.DMA_MemoryBaseAddr = (uint32_t)data_rx;
   DMA_Init(DMA_SPI1_RX_CH, &DMA_InitRX);

   /* Clear CE line before transfer (with delay)*/
   GPIO_ResetBits(SPI1_CE_GPIO_PORT, SPI1_CE_PIN);
   for (i=0; i<50; i++) { i=i; }

   /* Enable the DMA channels */
   DMA_Cmd(DMA_SPI1_TX_CH, ENABLE);
   DMA_Cmd(DMA_SPI1_RX_CH, ENABLE);

   /* Take access to SPI1 - will block until transfer finish */
   xSemaphoreTake(xSPI1Semaphore, portMAX_DELAY);
   xSemaphoreGive(xSPI1Semaphore);
}
