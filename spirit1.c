#include "spirit1.h"
#include <stm32l1xx.h>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <timers.h>
#include "messages.h"
#include "spi.h"
#include "MCU_Interface.h"
#include "SPIRIT_Config.h"
#include "options.h"
#include "ogn_lib.h"
#include "control.h"
#include "timer_const.h"

/* -------- defines -------- */
#define SPIRIT1_PKT_LEN     (3+2*(OGN_PKT_LEN)+1) // three bytes to complete the OGN SYNC word, 26 data+FEC bytes with Manchester emulation

#define SPR_SPI_MAX_REG_NUM  0xFF
#define SPR_SPI_HDR_LEN      2
#define SPR_MAX_FIFO_LEN     96

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

#define SPR1_GPIO0_PIN        GPIO_Pin_0
#define SPR1_GPIO0_GPIO_PORT  GPIOA
#define SPR1_GPIO0_GPIO_CLK   RCC_AHBPeriph_GPIOA
#define SPR1_GPIO0_PORT_SRC   EXTI_PortSourceGPIOA
#define SPR1_GPIO0_PIN_SRC    EXTI_PinSource0
#define SPR1_GPIO0_EXTI_LINE  EXTI_Line0
#define SPR1_GPIO0_EXTI_IRQ   EXTI0_IRQn

/*
Packet structure overview:

nRF905 sends the bit pattern 0xF50C9A93 at the beginning of every packet (0xF5 is hardwired and 0x0C9A93 is programmable in the chip).

Bits are sent at 50 kbps and are manchester encoded thus 0 is encoded as 10 and 1 as 01
Thus the nRF905 pattern 0xF5 looks like: 01010101 10011001 sent at 100 kbps
According to some sources nRF905 sends two more 1's thus at the beginning it really sends:
 1 1  1 1 1 1  0 1 0 1  : data       at  50 kbps
0101 01010101 10011001  : manchester at 100 kbps
as the preamble and then 24-bit address word 0x0C9A93 in the same manner.

The DVB-T dongle FLARM/OGN receiver does not care about the first two sync bits and it matched only the 32-bit SYNC: 0xF50C9A93
but just in case we want to send an nRF905 compatible signal, we should keep in mind the two bits at the very beginning.

For OGN we have chosen to send the inverted pattern thus 0x0AF3656C as it is more straightforward for Spirit1 chip
and makes clear distingtion from FLARM. At same time we send the OGN radio frames on a different frequency.

To send similar radio frames the Spirit1 RF chip is set to work in non-Manchester, 100 kbps data rate
and we emulate Manchester encoding in software.
Spirit1 sends 1-byte preamble which is 10101010 and it matches exactly the Manchester encoded 0 0 0 0.
Then we send Manchester encoded 0xAF36 as a 32-bit (thus 4 byte) Spirit sync-word.
After Manchester encoding 0xAF36 => 0x6655A596

When we send the packet, we encode the remaining part of the SYNC word: 0x56C as three bytes: 0x99 0x96 0x5A
and afterwards we encode 26 bytes of the apcket data.

*/

/* ------- declarations ------ */
void SP1_TX_packet(void);

/* -------- variables -------- */
xQueueHandle     xQueueSP1;
TimerHandle_t    xSP1Timer;

uint8_t Packet_TxBuff[SPR_MAX_FIFO_LEN], Packet_RxBuff[SPR_MAX_FIFO_LEN];
uint8_t Packet_TxBuff_Len;

uint8_t SPR_SPI_BufferTX[SPR_SPI_MAX_REG_NUM+SPR_SPI_HDR_LEN];
uint8_t SPR_SPI_BufferRX[SPR_SPI_MAX_REG_NUM+SPR_SPI_HDR_LEN];
/**
* @brief Radio structure fitting
*/
SRadioInit xRadioInit = {
   0,         // Xtal Offset:          0 ppm
   868.000e6, // Base Frequency:     868 MHz
   100e3,     // Channel spacing:    100 kHz
   4,         // Channel number:       4, frequency = 868.0+4*0.1 = 868.4MHz
   GFSK_BT05, // Modulation select: GFSK, BT=0.5
   100e3,     // Data rate:       99.976 kbps: Chapter 9.5.1: DATARATE_M=248 (spi1 011a00), DATARATE_E=11 (spi1 011b00).
              //                               Real speed is 2 times lower due to manchester code emulation.
   51e3,      // Freq Deviation: 50781.3 Hz  Chapter 9.5: FDEV_E=7, FDEV_M=0, (spi1 011c00)
   330e3      // Filter bandwidth: 325.4 kHz Table 33: CHFLT_M=6, CHFLT_E=1, (spi1 011d00)
};

/**
* @brief Packet Basic structure fitting
*/

PktBasicInit xBasicInit_OGN = { // for sending OGN packets
  PKT_PREAMBLE_LENGTH_01BYTE,   // 1 byte preamble
  PKT_SYNC_LENGTH_4BYTES,       // Spirit1 sync word 4 bytes
  // 0x6655A596,                   // Spirit1 sync word = the first two OGN SYNC bytes Manchester encoded
  0xA6655A59,                   // here we add extra 2 data bits (thus 4 Manchester bits) to make preamble longer - nRF905-like
  PKT_LENGTH_FIX,               // fixed length packet
  7,                            // optional field that is defined as the cumulative length of Address, Control, and Payload fields
                                // in fixed length mode, the field length is not used
  PKT_NO_CRC,                   // no CRC fields - we make the error checking and correcting code
  PKT_CONTROL_LENGTH_0BYTES,    // 
  S_DISABLE,                    // no address field
  S_DISABLE,                    // no FEC
  S_DISABLE                     // no data whitening
};

/**
* @brief GPIO structure fitting
*/
SGpioInit xGpioIRQ={
  SPIRIT_GPIO_0,
  SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,
  SPIRIT_GPIO_DIG_OUT_IRQ
};

/* -------- constants -------- */
/* Spirit1 Manchester encoding emulation */
/* 0 encoded as 1->0 (negative) transition: 10 */
/* 1 encoded as 0->1 (positive) transition: 01 */
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

/* Code to generate the Manchester decoding table (compiles with warnings)

#include <stdio.h>
#include <stdint.h>

int main(int argc, char *argv[])
{
  uint8_t Decode[256];

  for(uint8_t Idx=0; ; Idx++)
  { uint8_t Byte=Idx; uint8_t Code=0;
    for(int BitIdx=0; BitIdx<4; BitIdx++)
    { uint8_t DiBit=Byte&0x03;
      uint8_t Bit; uint8_t Err;
      switch(DiBit)
      { case 0: Bit=0; Err=1; break;
        case 1: Bit=1; Err=0; break;
        case 2: Bit=0; Err=0; break;
        case 3: Bit=1; Err=1; break; }
      Code>>=1; Code |= (Err<<7) | (Bit<<3);
      Byte>>=2; }
     Decode[Idx]=Code;
     if(Idx==255) break; }

  printf("const uint8_t manch_2_hex_to_trans[256] =\n{");
  for(uint8_t Idx=0; ; Idx++)
  { if((Idx&0x0F)==0) printf("\n ");
    printf(" 0x%02X", Decode[Idx]);
    if(Idx!=255) printf(",");
            else break;
  }
  printf("\n} ;\n");

  return 0; }

*/

const uint8_t manch_2_hex_to_trans[256] = // lower nibble has the data bits and the upper nibble the error pattern
{
  0xF0, 0xE1, 0xE0, 0xF1, 0xD2, 0xC3, 0xC2, 0xD3, 0xD0, 0xC1, 0xC0, 0xD1, 0xF2, 0xE3, 0xE2, 0xF3,
  0xB4, 0xA5, 0xA4, 0xB5, 0x96, 0x87, 0x86, 0x97, 0x94, 0x85, 0x84, 0x95, 0xB6, 0xA7, 0xA6, 0xB7,
  0xB0, 0xA1, 0xA0, 0xB1, 0x92, 0x83, 0x82, 0x93, 0x90, 0x81, 0x80, 0x91, 0xB2, 0xA3, 0xA2, 0xB3,
  0xF4, 0xE5, 0xE4, 0xF5, 0xD6, 0xC7, 0xC6, 0xD7, 0xD4, 0xC5, 0xC4, 0xD5, 0xF6, 0xE7, 0xE6, 0xF7,
  0x78, 0x69, 0x68, 0x79, 0x5A, 0x4B, 0x4A, 0x5B, 0x58, 0x49, 0x48, 0x59, 0x7A, 0x6B, 0x6A, 0x7B,
  0x3C, 0x2D, 0x2C, 0x3D, 0x1E, 0x0F, 0x0E, 0x1F, 0x1C, 0x0D, 0x0C, 0x1D, 0x3E, 0x2F, 0x2E, 0x3F,
  0x38, 0x29, 0x28, 0x39, 0x1A, 0x0B, 0x0A, 0x1B, 0x18, 0x09, 0x08, 0x19, 0x3A, 0x2B, 0x2A, 0x3B,
  0x7C, 0x6D, 0x6C, 0x7D, 0x5E, 0x4F, 0x4E, 0x5F, 0x5C, 0x4D, 0x4C, 0x5D, 0x7E, 0x6F, 0x6E, 0x7F,
  0x70, 0x61, 0x60, 0x71, 0x52, 0x43, 0x42, 0x53, 0x50, 0x41, 0x40, 0x51, 0x72, 0x63, 0x62, 0x73,
  0x34, 0x25, 0x24, 0x35, 0x16, 0x07, 0x06, 0x17, 0x14, 0x05, 0x04, 0x15, 0x36, 0x27, 0x26, 0x37,
  0x30, 0x21, 0x20, 0x31, 0x12, 0x03, 0x02, 0x13, 0x10, 0x01, 0x00, 0x11, 0x32, 0x23, 0x22, 0x33,
  0x74, 0x65, 0x64, 0x75, 0x56, 0x47, 0x46, 0x57, 0x54, 0x45, 0x44, 0x55, 0x76, 0x67, 0x66, 0x77,
  0xF8, 0xE9, 0xE8, 0xF9, 0xDA, 0xCB, 0xCA, 0xDB, 0xD8, 0xC9, 0xC8, 0xD9, 0xFA, 0xEB, 0xEA, 0xFB,
  0xBC, 0xAD, 0xAC, 0xBD, 0x9E, 0x8F, 0x8E, 0x9F, 0x9C, 0x8D, 0x8C, 0x9D, 0xBE, 0xAF, 0xAE, 0xBF,
  0xB8, 0xA9, 0xA8, 0xB9, 0x9A, 0x8B, 0x8A, 0x9B, 0x98, 0x89, 0x88, 0x99, 0xBA, 0xAB, 0xAA, 0xBB,
  0xFC, 0xED, 0xEC, 0xFD, 0xDE, 0xCF, 0xCE, 0xDF, 0xDC, 0xCD, 0xCC, 0xDD, 0xFE, 0xEF, 0xEE, 0xFF
} ;


/* -------- interrupt handlers -------- */

/* interrupt for raising GPIO0 line */
void EXTI0_IRQHandler(void)
{
   task_message sp1_msg;
   portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
   
   if(EXTI_GetITStatus(SPR1_GPIO0_EXTI_LINE) != RESET)
   {
        /* Clear the GPIO0 EXTI line pending bit */
        EXTI_ClearITPendingBit(SPR1_GPIO0_EXTI_LINE);
        sp1_msg.msg_data   = 0;
        sp1_msg.msg_len    = 0;
        sp1_msg.msg_opcode = SP1_INT_GPIO0_IRQ;
        sp1_msg.src_id     = SPIRIT1_SRC_ID;
        xQueueSendFromISR(xQueueSP1, &sp1_msg, &xHigherPriorityTaskWoken);
   }
   portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/* -------- functions -------- */
xQueueHandle* Get_SP1Queue()
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
* @brief  SP1 timer callback.
* @param  None
* @retval None
*/
void vSP1TimerCallback(TimerHandle_t pxTimer)
{
   SP1_TX_packet();
}

/**
* @brief  Configures the SPIRIT1 IC.
* @param  None
* @retval None
*/
void Spirit1_Config(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
   EXTI_InitTypeDef EXTI_InitStructure;

   xSP1Timer = xTimerCreate(
      "SP1Timer",
      /* The timer period in ms. */
      TIMER_MS(200),
      /* The timer will stop when expire. */
      pdFALSE,
      /* unique id */
      ( void * )SP1_TIMER_ID,
      /* Each timer calls the same callback when it expires. */
      vSP1TimerCallback
    );
    
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

   /* SPIRIT1 GPIO0 pin configuration */
   RCC_AHBPeriphClockCmd(SPR1_GPIO0_GPIO_CLK, ENABLE);

   GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
   GPIO_InitStructure.GPIO_Pin   = SPR1_GPIO0_PIN;
   GPIO_Init(SPR1_GPIO0_GPIO_PORT, &GPIO_InitStructure);

   SYSCFG_EXTILineConfig(SPR1_GPIO0_PORT_SRC, SPR1_GPIO0_PIN_SRC);

   EXTI_InitStructure.EXTI_Line    = SPR1_GPIO0_EXTI_LINE;
   EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
   EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
   EXTI_InitStructure.EXTI_LineCmd = ENABLE;
   EXTI_Init(&EXTI_InitStructure);

   /* No packet in TX buffer */
   Packet_TxBuff_Len = 0;

}

/**
* @brief  Function called when Spirit1 IC not detected.
* @param  None
* @retval None
*/
void static SpiritNotPresent(void)
{
   for (;;)
   {
      vTaskDelay(1000);
   }
}

/**
* @brief  Configure TX parameters.
* @param  None
* @retval None
*/
void static SpiritTXConf(float TxPower)
{
   /* maximum output power was measured for SPIRIT1_LIB_MAX_POWER setting */
   float max_power = *(float *)GetOption(OPT_MAX_TX_PWR);
   float pwr_delta = max_power - SPIRIT1_LIB_MAX_POWER;
   float lib_power = TxPower - pwr_delta;

   if (lib_power > SPIRIT1_LIB_MAX_POWER) lib_power = SPIRIT1_LIB_MAX_POWER;
   if (lib_power < SPIRIT1_LIB_MIN_POWER) lib_power = SPIRIT1_LIB_MIN_POWER;

   /* Spirit Radio set power */
   SpiritRadioSetPALeveldBm(0, lib_power);
   SpiritRadioSetPALevelMaxIndex(0);
}

/**
* @brief  Copy OGN packet to Spirit1 task memory.
* @param  packet length and address, could be invalid when packet data should be cleared
* @retval None
*/
void SpiritCopyPacket_OGN(const uint8_t* pkt_data, uint8_t pkt_len)
{
   uint8_t in_pkt_pos, out_pkt_pos = 0;
   if ((pkt_data)&&(pkt_len))
   {
      uint8_t Buff = 0x06; uint8_t Byte;     // complete the preamble/SYNC
      Byte = hex_2_manch_encoding[0x5]; Buff = (Buff<<4) | (Byte>>4); Packet_TxBuff[out_pkt_pos++] = Buff; Buff = Byte&0x0F;
      Byte = hex_2_manch_encoding[0x6]; Buff = (Buff<<4) | (Byte>>4); Packet_TxBuff[out_pkt_pos++] = Buff; Buff = Byte&0x0F;
      Byte = hex_2_manch_encoding[0xC]; Buff = (Buff<<4) | (Byte>>4); Packet_TxBuff[out_pkt_pos++] = Buff; Buff = Byte&0x0F;
      for (in_pkt_pos = 0; in_pkt_pos<pkt_len; in_pkt_pos++)
      { uint8_t Data = pkt_data[in_pkt_pos]; // get the next data byte => convert in two Manchester bytes
        Byte = hex_2_manch_encoding[Data>>4];   Buff = (Buff<<4) | (Byte>>4); Packet_TxBuff[out_pkt_pos++] = Buff; Buff = Byte&0x0F;
        Byte = hex_2_manch_encoding[Data&0x0F]; Buff = (Buff<<4) | (Byte>>4); Packet_TxBuff[out_pkt_pos++] = Buff; Buff = Byte&0x0F;
      }
      Buff = (Buff<<4) | 0x0A; Packet_TxBuff[out_pkt_pos++] = Buff;
   }
   Packet_TxBuff_Len = out_pkt_pos;
}

/**
* @brief  Transmit prepared packet.
* @param  None
* @retval None
*/
void SP1_TX_packet(void)
{
    if (Packet_TxBuff_Len)
    {
        SpiritCmdStrobeFlushTxFifo();
        SpiritSpiWriteLinearFifo(SPIRIT1_PKT_LEN, Packet_TxBuff);
        SpiritCmdStrobeTx();
    }
}

/**
* @brief  Transmit prepared packet with delay and LBT.
* @param  None
* @retval None
*/
void SP1_TX_packet_LBT(uint32_t max_tx_delay_ms)
{
    uint16_t tx_delay = rand() % max_tx_delay_ms;
    xTimerChangePeriod(xSP1Timer, TIMER_MS(tx_delay), portMAX_DELAY);
    xTimerStart(xSP1Timer, portMAX_DELAY);
}


/* Data for single packet */
rcv_packet_str rcv_packet;
uint8_t ogn_packet_data[OGN_PKT_LEN];
uint8_t ogn_packet_err[OGN_PKT_LEN];

/**
* @brief  Receive OGN packet.
* @param  None
* @retval None
*/
rcv_packet_str* SpiritReceivePacket_OGN(void)
{
    uint16_t cRxData;
    uint8_t  in_pkt_pos, out_pkt_pos;

    cRxData = SpiritLinearFifoReadNumElementsRxFifo();
    SpiritSpiReadLinearFifo(cRxData, Packet_RxBuff);
    /* Flush the RX FIFO */
    SpiritCmdStrobeFlushRxFifo();

    if (cRxData != SPIRIT1_PKT_LEN) return NULL;

    rcv_packet.data_ptr = ogn_packet_data;
    rcv_packet.err_ptr  = ogn_packet_err;
    rcv_packet.rssi     = SpiritQiGetRssidBm();
    rcv_packet.lqi      = SpiritQiGetLqi();
    rcv_packet.pqi      = SpiritQiGetPqi();
    rcv_packet.sqi      = SpiritQiGetSqi();

    // Decode Manchester
    in_pkt_pos = 0; uint8_t Manch, Data, Err;
    Manch = Packet_RxBuff[in_pkt_pos++]; Data=manch_2_hex_to_trans[Manch]; Err=Data>>4; Data&=0x0F;
    Manch = Packet_RxBuff[in_pkt_pos++]; Data=manch_2_hex_to_trans[Manch]; Err=Data>>4; Data&=0x0F;
    Manch = Packet_RxBuff[in_pkt_pos++]; Data=manch_2_hex_to_trans[Manch]; Err=Data>>4; Data&=0x0F;
    Manch = Packet_RxBuff[in_pkt_pos++]; Data=manch_2_hex_to_trans[Manch]; Err=Data>>4; Data&=0x0F;

    for (out_pkt_pos = 0; out_pkt_pos < OGN_PKT_LEN; out_pkt_pos++)
    { uint8_t DataByte = Data; uint8_t ErrByte=Err;
      Manch = Packet_RxBuff[in_pkt_pos++]; Data=manch_2_hex_to_trans[Manch]; Err=Data>>4; Data&=0x0F;
      DataByte = (DataByte<<4) |  Data;     ErrByte = (ErrByte<<4) |  Err;
      Manch = Packet_RxBuff[in_pkt_pos++]; Data=manch_2_hex_to_trans[Manch]; Err=Data>>4; Data&=0x0F;
      DataByte = (DataByte<<2) | (Data>>2); ErrByte = (ErrByte<<2) | (Err>>2);
      ogn_packet_data[out_pkt_pos] = DataByte; ogn_packet_err[out_pkt_pos] = ErrByte; }

    return &rcv_packet;
}


/**
* @brief  Switch Spirit1 to CW transmitting mode.
* @param  None
* @retval None
*/
void SP1_Enter_CW_mode(void)
{
   SpiritCmdStrobeSabort();
   SpiritDirectRfSetTxMode(PN9_TX_MODE);
   SpiritRadioCWTransmitMode(S_ENABLE);
   SpiritCmdStrobeTx();
}

/**
* @brief  Exit Spirit1 from CW transmitting mode.
* @param  None
* @retval None
*/
void SP1_Leave_CW_mode(void)
{
   SpiritCmdStrobeSabort();
   SpiritDirectRfSetTxMode(NORMAL_TX_MODE);
   SpiritRadioCWTransmitMode(S_DISABLE);
}

/**
* @brief  Switch Spirit1 to persistent RX mode.
* @param  None
* @retval None
*/
void SP1_Enter_Pers_RX_mode(void)
{
   SpiritRadioPersistenRx(S_ENABLE);
   SpiritCmdStrobeRx();
}

void vTaskSP1(void* pvParameters)
{
   task_message msg, control_msg;
   SpiritIrqs xIrqStatus;
   xQueueHandle* control_queue;
   rcv_packet_str* rcv_packet_ptr;
   NVIC_InitTypeDef NVIC_InitStructure;
    
   Spirit1ExitShutdown();

   if (SpiritGeneralGetDevicePartNumber() != 0x0130)
   {
      /* if Spirit not present stop here */
      SpiritNotPresent();
   }

   SpiritRadioSetXtalFrequency(26e6);
   SpiritGeneralSetSpiritVersion(SPIRIT_VERSION_3_0);

    /* Spirit IRQ configuration */
   SpiritGpioInit(&xGpioIRQ);

   xRadioInit.nXtalOffsetPpm  = *(int16_t *)GetOption(OPT_XTAL_CORR);
   xRadioInit.lFrequencyBase += *(int32_t *)GetOption(OPT_FREQ_OFS);
   xRadioInit.cChannelNumber  = *(uint8_t *)GetOption(OPT_CHANNEL);
   SpiritRadioInit(&xRadioInit);

   SpiritPktBasicInit(&xBasicInit_OGN);

   /* payload length configuration */
   SpiritPktBasicSetPayloadLength(SPIRIT1_PKT_LEN);

   /* Set TX parameters */
   SpiritTXConf(*(float *)GetOption(OPT_TX_POWER));

   /* Spirit IRQs enable */
   SpiritIrqDeInit(NULL);
   SpiritIrq(RX_DATA_READY, S_ENABLE);

   /* RX timeout config */
   SpiritTimerSetRxTimeoutMs(500.0);

   /* QI Config */
   SpiritQiSqiCheck(S_ENABLE);
   SpiritQiSetSqiThreshold(SQI_TH_2); /* 4 wrong bits in sync accepted */

   /* IRQ registers blanking */
   SpiritIrqClearStatus();

   /* Get Control task queue */
   control_queue = Get_ControlQueue();

   /* Create queue for SP1 task messages received */
   xQueueSP1 = xQueueCreate(10, sizeof(task_message));

   /* enable Spirit1 GPIO0 input line interrupt */
   NVIC_InitStructure.NVIC_IRQChannel = SPR1_GPIO0_EXTI_IRQ;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configSPIRIT1_INTERRUPT_PRIORITY;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);

   for(;;)
   {
      xQueueReceive(xQueueSP1, &msg, portMAX_DELAY);
      switch (msg.msg_opcode)
      {
         case SP1_COPY_OGN_PKT:            // a request to copy a packet data
            SpiritCopyPacket_OGN((uint8_t*)msg.msg_data, msg.msg_len);
            break;
         case SP1_CHG_CHANNEL:             // a request to change active channel
            xTimerStop(xSP1Timer, portMAX_DELAY); // cancel running TX timer (if not expired already) 
            SpiritCmdStrobeSabort();       // cancel all activities
            SpiritRadioSetChannel(msg.msg_data);
            break;
         case SP1_START_CW:                // a request to start CW
            SP1_Enter_CW_mode();
            break;
         case SP1_STOP_CW:                 // a request to stop CW
            SP1_Leave_CW_mode();
            break;
         case SP1_START_RX:                // a request to start RX
            SP1_Enter_Pers_RX_mode();
            break;
         case SP1_TX_PACKET:               // a request to TX buffered packet
            SP1_TX_packet();
            break;
         case SP1_TX_PACKET_LBT:           // a request to TX with LBT buffered packet
            SP1_TX_packet_LBT(msg.msg_data);
            break;
         case SP1_INT_GPIO0_IRQ:           // internal message - GPIO0 triggered
         {
            /* Check/clear interrupt status register */
            SpiritIrqGetStatus(&xIrqStatus);
            /* Check RX Data Ready IRQ */
            if (xIrqStatus.IRQ_RX_DATA_READY)
            {
                /* Attempt to receive OGN packet */
                rcv_packet_ptr = SpiritReceivePacket_OGN();
                if (rcv_packet_ptr && control_queue)
                {
                    /* Send received packet to control task */
                    control_msg.msg_data   = (uint32_t)rcv_packet_ptr;
                    control_msg.msg_len    = 0;
                    control_msg.msg_opcode = SP1_OUT_PKT_READY;
                    control_msg.src_id     = SPIRIT1_SRC_ID;
                    xQueueSend(*control_queue, &control_msg, portMAX_DELAY);
                }
            }
            break;
         }
         default:
            break;
      }
   }
}
