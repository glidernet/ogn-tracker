#include "spirit1.h"
#include <stm32l1xx.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include "messages.h"
#include "spi.h"
#include "MCU_Interface.h"
#include "SPIRIT_Config.h"

/* -------- defines -------- */
#define SPR_SPI_MAX_REG_NUM  0xFF
#define SPR_SPI_HDR_LEN      2

/** @defgroup SPI_Headers
* @{
*/
#define HEADER_WRITE_MASK     0x00 /*!< Write mask for header byte*/
#define HEADER_READ_MASK      0x01 /*!< Read mask for header byte*/
#define HEADER_ADDRESS_MASK   0x00 /*!< Address mask for header byte*/
#define HEADER_COMMAND_MASK   0x80 /*!< Command mask for header byte*/

#define LINEAR_FIFO_ADDRESS   0xFF  /*!< Linear FIFO address*/

/** @defgroup SPI_Private_Macros
* @{
*/
#define BUILT_HEADER(add_comm, w_r) (add_comm | w_r)  /*!< macro to build the header byte*/
#define WRITE_HEADER    BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_WRITE_MASK) /*!< macro to build the write header byte*/
#define READ_HEADER     BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_READ_MASK)  /*!< macro to build the read header byte*/
#define COMMAND_HEADER  BUILT_HEADER(HEADER_COMMAND_MASK, HEADER_WRITE_MASK) /*!< macro to build the command header byte*/

/* - SPIRIT1 pins mappings - */
#define SPR1_SHDN_PIN        GPIO_Pin_8
#define SPR1_SHDN_GPIO_PORT  GPIOA
#define SPR1_SHDN_GPIO_CLK   RCC_AHBPeriph_GPIOA

/* -------- variables -------- */
xQueueHandle     xQueueSP1;

uint8_t SPR_SPI_BufferTX[SPR_SPI_MAX_REG_NUM+SPR_SPI_HDR_LEN];
uint8_t SPR_SPI_BufferRX[SPR_SPI_MAX_REG_NUM+SPR_SPI_HDR_LEN];
/**
* @brief Radio structure fitting
*/
SRadioInit xRadioInit = {
   0,         // Xtal Offset
   868.000e6, // Base Frequency
   100e3,     // Channel space
   4,         // Channel number
   GFSK_BT05, // Modulation select
   100e3,     // Data rate
   51e3,      // Freq Deviation
   330e3      // Filter bandwidth
};


/* -------- interrupt handlers -------- */

/* -------- functions -------- */
StatusBytes SPI1WriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
   uint8_t i;
   static uint16_t tmpstatus;

   StatusBytes *status=(StatusBytes *)&tmpstatus;

   /* Write header */
   SPR_SPI_BufferTX[0] = WRITE_HEADER;
   SPR_SPI_BufferTX[1] = cRegAddress;

   for (i = 0; i<cNbBytes ; i++)  SPR_SPI_BufferTX[2+i] = pcBuffer[i];
   SPI1_Send(SPR_SPI_BufferTX, SPR_SPI_BufferRX, cNbBytes+2);
   tmpstatus = ((uint16_t)SPR_SPI_BufferRX[0]<<8) | SPR_SPI_BufferRX[1];

   return *status;
}

StatusBytes SPI1CommandStrobes(uint8_t cCommandCode)
{
   static uint16_t tmpstatus;
   StatusBytes *status=(StatusBytes *)&tmpstatus;

   /* Command header */
   SPR_SPI_BufferTX[0] = COMMAND_HEADER;
   SPR_SPI_BufferTX[1] = cCommandCode;

   SPI1_Send(SPR_SPI_BufferTX, SPR_SPI_BufferRX, 2);

   tmpstatus = ((uint16_t)SPR_SPI_BufferRX[0]<<8) | SPR_SPI_BufferRX[1];

   return *status;
}

StatusBytes SPI1ReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
   uint8_t i;
   static uint16_t tmpstatus;
   StatusBytes *status=(StatusBytes *)&tmpstatus;

   /* Read header */
   SPR_SPI_BufferTX[0] = READ_HEADER;
   SPR_SPI_BufferTX[1] = cRegAddress;

   SPI1_Send(SPR_SPI_BufferTX, SPR_SPI_BufferRX, cNbBytes+2);

   tmpstatus = ((uint16_t)SPR_SPI_BufferRX[0]<<8) | SPR_SPI_BufferRX[1];
   for (i = 0; i<cNbBytes ; i++)  pcBuffer[i] = SPR_SPI_BufferRX[2+i];

   return *status;
}

/**
 * @brief  Puts at logic 1 the SDN pin.
 * @param  None.
 * @retval None.
 */
void Spirit1EnterShutdown(void)
{
   /* Puts high the GPIO connected to shutdown pin */
   GPIO_SetBits(SPR1_SHDN_GPIO_PORT, SPR1_SHDN_PIN);
}

/**
 * @brief  Put at logic 0 the SDN pin.
 * @param  None.
 * @retval None.
 */
void Spirit1ExitShutdown(void)
{
  /* Puts low the GPIO connected to shutdown pin */
  GPIO_ResetBits(SPR1_SHDN_GPIO_PORT, SPR1_SHDN_PIN);

  /* Delay to allow the circuit POR, about 700 us */
  vTaskDelay(10);
}

/**
* @brief  Configures the SPIRIT1 IC.
* @param  None
* @retval None
*/
void Spirit1_Config(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;

   SPI1_Config();

   /* SPIRIT1 SHDN pin configuration */
   RCC_AHBPeriphClockCmd(SPR1_SHDN_GPIO_CLK, ENABLE);

   GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
   GPIO_InitStructure.GPIO_Pin   = SPR1_SHDN_PIN;
   GPIO_Init(SPR1_SHDN_GPIO_PORT, &GPIO_InitStructure);

   Spirit1EnterShutdown();
}

void SpiritNotPresent(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;

   /* SPI1 SCK is connected to LED, change it to GPIO */
   GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
   GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;

   GPIO_Init(GPIOA, &GPIO_InitStructure);

   for (;;)
   {
      GPIO_SetBits(GPIOA, GPIO_Pin_5);
      vTaskDelay(150);
      GPIO_ResetBits(GPIOA, GPIO_Pin_5);
      vTaskDelay(850);
   }
}

void vTaskSP1(void* pvParameters)
{
   task_message msg;

   Spirit1ExitShutdown();

   if (SpiritGeneralGetDevicePartNumber() != 0x0130)
   {
      /* if Spirit not present stop here */
      SpiritNotPresent();
   }

   SpiritRadioSetXtalFrequency(26e6);
   SpiritGeneralSetSpiritVersion(SPIRIT_VERSION_3_0);

   SpiritRadioInit(&xRadioInit);

   /* Create queue for SP1 task messages received */
   xQueueSP1 = xQueueCreate(10, sizeof(task_message));

   for(;;)
   {
      xQueueReceive(xQueueSP1, &msg, portMAX_DELAY);
   }
}
