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

#include "options.h"

/* -------- defines -------- */
#define SPIRIT1_PKT_LEN     (2*(2+26)) // two bytes to complete the SYNC, 26 data bytes and times two because we emulate Manchester encoding

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

uint8_t Packet_TxBuff[96];

uint8_t SPR_SPI_BufferTX[SPR_SPI_MAX_REG_NUM+SPR_SPI_HDR_LEN];
uint8_t SPR_SPI_BufferRX[SPR_SPI_MAX_REG_NUM+SPR_SPI_HDR_LEN];
/**
* @brief Radio structure fitting
*/
SRadioInit xRadioInit = {
   0,         // Xtal Offset
   868.000e6, // Base Frequency
   100e3,     // Channel spacing (could be
   4,         // Channel number: the frequency is 868.0+4*0.1 = 868.4MHz
   GFSK_BT05, // Modulation select
   100e3,     // Data rate
   51e3,      // Freq Deviation
   330e3      // Filter bandwidth
};

/**
* @brief Packet Basic structure fitting
*/
PktBasicInit xBasicInit={
  PKT_PREAMBLE_LENGTH_02BYTES,
  PKT_SYNC_LENGTH_3BYTES,
  0x6655A596,      // sync word = the first two bytes with Macnhester encoding emulation
  PKT_LENGTH_FIX,  // fixed length packet
  7,               /* length width */
  PKT_NO_CRC,      // no CRC fields - we make the error checking and correcting code
  PKT_CONTROL_LENGTH_0BYTES,
  S_DISABLE,       /* no address field */
  S_DISABLE,       /* no FEC */
  S_DISABLE        /* no data whitening */
};
/* -------- constants -------- */
/* Spirit1 Manchester encoding simulation */
/* 0 encoded as 1->0 transition */
/* 1 encoded as 0->1 transition */
const uint8_t hex_2_manch_encoding[0x10] =         // lookup table for 4-bit nibbles
{
   0xAA, /* hex: 0, bin: 0000, manch: 10101010 */
   0xA9, /* hex: 1, bin: 0001, manch: 10101001 */
   0xA6, /* hex: 2, bin: 0010, manch: 10100110 */
   0xA5, /* hex: 3, bin: 0011, manch: 10100101 */
   0x9A, /* hex: 4, bin: 0100, manch: 10011010 */
   0x99, /* hex: 5, bin: 0101, manch: 10011001 */
   0x96, /* hex: 6, bin: 0110, manch: 10010110 */
   0x95, /* hex: 7, bin: 0111, manch: 10010101 */
   0x6A, /* hex: 8, bin: 1000, manch: 01101010 */
   0x69, /* hex: 9, bin: 1001, manch: 01101001 */
   0x66, /* hex: A, bin: 1010, manch: 01100110 */
   0x65, /* hex: B, bin: 1011, manch: 01100101 */
   0x5A, /* hex: C, bin: 1100, manch: 01011010 */
   0x59, /* hex: D, bin: 1101, manch: 01011001 */
   0x56, /* hex: E, bin: 1110, manch: 01010110 */
   0x55  /* hex: F, bin: 1111, manch: 01010101 */
};

/* -------- interrupt handlers -------- */

/* -------- functions -------- */
xQueueHandle* Get_SP1Que()
{
   return &xQueueSP1;
}

/**
* @brief  Function handles Spirit1 DK errors.
* @param  SPI1 library error code
* @retval None
*/
void SpiritReportError(spi1_err_types code)
{
	/* should report error and block */
	for (;;) {}
}

StatusBytes SPI1WriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
   uint8_t i;
   static uint16_t tmpstatus;

   StatusBytes* status=(StatusBytes*)&tmpstatus;

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
   StatusBytes* status=(StatusBytes*)&tmpstatus;

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
   StatusBytes* status=(StatusBytes*)&tmpstatus;

   /* Read header */
   SPR_SPI_BufferTX[0] = READ_HEADER;
   SPR_SPI_BufferTX[1] = cRegAddress;

   SPI1_Send(SPR_SPI_BufferTX, SPR_SPI_BufferRX, cNbBytes+2);

   tmpstatus = ((uint16_t)SPR_SPI_BufferRX[0]<<8) | SPR_SPI_BufferRX[1];
   for (i = 0; i<cNbBytes ; i++)  pcBuffer[i] = SPR_SPI_BufferRX[2+i];

   return *status;
}

StatusBytes SPI1WriteFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
   uint8_t i;
   static uint16_t tmpstatus;
   StatusBytes* status=(StatusBytes*)&tmpstatus;

   /* Write header */
   SPR_SPI_BufferTX[0] = WRITE_HEADER;
   SPR_SPI_BufferTX[1] = LINEAR_FIFO_ADDRESS;

   for (i = 0; i<cNbBytes ; i++)  SPR_SPI_BufferTX[2+i] = pcBuffer[i];

   SPI1_Send(SPR_SPI_BufferTX, SPR_SPI_BufferRX, cNbBytes+2);

   tmpstatus = ((uint16_t)SPR_SPI_BufferRX[0]<<8) | SPR_SPI_BufferRX[1];

   return *status;
}

StatusBytes SPI1ReadFifo(uint8_t cNbBytes, uint8_t* pcBuffer)
{
   uint8_t i;
   static uint16_t tmpstatus;
   StatusBytes* status=(StatusBytes*)&tmpstatus;

   /* Read header */
   SPR_SPI_BufferTX[0] = READ_HEADER;
   SPR_SPI_BufferTX[1] = LINEAR_FIFO_ADDRESS;

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

/**
* @brief  Function called when Spirit1 IC not detected.
* @param  None
* @retval None
*/
void static SpiritNotPresent(void)
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

/**
* @brief  Configure TX parameters.
* @param  None
* @retval None
*/
void static SpiritTXConf(float TxPower)
{
   /* Spirit Radio set power */
   SpiritRadioSetPALeveldBm(0, TxPower);
   SpiritRadioSetPALevelMaxIndex(0);
}

void static SpiritTxPower(float TxPower) { SpiritRadioSetPALeveldBm(0, TxPower); }

/**
* @brief  Send OGN packet.
* @param  None
* @retval None
*/
void SpiritSendOGNPacket(uint8_t* pkt_data, uint8_t pkt_len)
{
   uint8_t in_pkt_pos, out_pkt_pos = 0;

   /* Fill end of sync word (sync start in SP1 sync register) */
   Packet_TxBuff[out_pkt_pos++] = 0x96;
   Packet_TxBuff[out_pkt_pos++] = 0x99;
   Packet_TxBuff[out_pkt_pos++] = 0x96;
   Packet_TxBuff[out_pkt_pos++] = 0x5A;

   for (in_pkt_pos = 0; in_pkt_pos<pkt_len; in_pkt_pos++)
   {
      uint8_t up_half = pkt_data[in_pkt_pos]>>4;
      uint8_t lo_half = pkt_data[in_pkt_pos]&0x0F;
      Packet_TxBuff[out_pkt_pos++] = hex_2_manch_encoding[up_half];
      Packet_TxBuff[out_pkt_pos++] = hex_2_manch_encoding[lo_half];
   }

   SpiritCmdStrobeFlushTxFifo();
   SpiritSpiWriteLinearFifo(SPIRIT1_PKT_LEN, Packet_TxBuff);

   SpiritCmdStrobeTx();
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

   xRadioInit.nXtalOffsetPpm  = *(int16_t *)GetOption(OPT_XTAL_CORR);
   xRadioInit.lFrequencyBase += *(int32_t *)GetOption(OPT_FREQ_OFS);
   SpiritRadioInit(&xRadioInit);

   /* Spirit Packet config */
   SpiritPktBasicInit(&xBasicInit);

   /* payload length config */
   SpiritPktBasicSetPayloadLength(SPIRIT1_PKT_LEN);

   /* Set TX parameters */
   SpiritTXConf(*(float *)GetOption(OPT_TX_POWER));

   /* Create queue for SP1 task messages received */
   xQueueSP1 = xQueueCreate(10, sizeof(task_message));

   for(;;)
   {
      xQueueReceive(xQueueSP1, &msg, portMAX_DELAY);
      switch (msg.msg_opcode)
      {
         case SP1_SEND_OGN_PKT:             // a request to send a packet
            SpiritTxPower(*(float *)GetOption(OPT_TX_POWER));
            SpiritSendOGNPacket((uint8_t*)msg.msg_data, msg.msg_len);
            break;
         case SP1_GPS_FIRST_NMEA:           // a signal that the GPS started the transmission (about 0.3sec after a PPS)
            break;
         default:
            break;
      }
   }
}
