#include "commands.h"

#include <stm32l1xx.h>
#include <stdio.h>
#include <stdlib.h>

#include <FreeRTOS.h>
#include <FreeRTOS_CLI.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include <string.h>
#include <math.h>

#include "options.h"
#include "messages.h"
#include "spi.h"
#include "spirit1.h"
#include "gps.h"


/* -------- defines -------- */
#define SPI_DATA_LEN 256
#define ACFT_ID_LEN    4

/* -------- variables -------- */
uint8_t SPI1_tx_data[SPI_DATA_LEN];
uint8_t SPI1_rx_data[SPI_DATA_LEN];

uint8_t OGN_packet[OGN_PKT_LEN];

/* -------- constants -------- */
static const char * const pcVersion = "0.0.2";
/* -------- functions -------- */

/**
  * @brief  Get value from single 0-F hex digit
  * @param  hex char
  * @retval integer value, -1 if not hex at input
  */
static int8_t get_hex_val(char chr)
{ if ((chr >= '0') && (chr <= '9')) return chr-'0';
  if ((chr >= 'A') && (chr <= 'F')) return chr-'A'+0x0A;
  if ((chr >= 'a') && (chr <= 'f')) return chr-'a'+0x0A;
  return -1; }

/**
  * @brief  Converts a two digit hex string to byte
  * @param  two hex chars (non-valid characters replaced with zero)
  * @retval integer value.
  */
static uint8_t get_hex_str_val(const char* str)
{ int8_t High = get_hex_val(str[0]); if(High<0) High=0;
  int8_t Low  = get_hex_val(str[1]); if(Low<0)  Low =0;
  return (High<<4) | Low; }

/**
  * @brief  Converts a string to hex
  * @param  a string of hex digits (convert stops at first non-hex digit)
  * @retval integer value.
  */
/* not used for now: strtol() is likely easier to use
static uint32_t get_hex_uint32(const char* str)
{ uint32_t Value=0;
  for( ; ; )
  { int8_t hex=get_hex_val(*str++); if(hex<0) break;
    Value = (Value<<4) | hex; }
  return Value; }
*/

/**
  * @brief  Prints one byte hex value to string
  * @param  hex value, pointer to output string
  * @retval number of chars copied to string.
  */
static uint8_t print_hex_val(uint8_t data, char* dest)
{
   char t;
   t = data>>4;
   dest[0] = t > 9 ? t+'A'-0x0A : t+'0';
   t = data&0x0F;
   dest[1] = t > 9 ? t+'A'-0x0A : t+'0';
   return 2;
}

// ---------------------------------------------------------------------------------------------------------------------------

/**
  * @brief  Command ver: prints version string.
  * @param  CLI template
  * @retval CLI template
  */
static portBASE_TYPE prvVerCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ uint32_t *ID = (uint32_t*)0x1FF80050;
  sprintf(pcWriteBuffer, "Soft. version: %s\r\nMCU ID: %08lX %08lX %08lX\r\n", pcVersion, ID[0], ID[1], ID[2]);
  return pdFALSE; }

/**
  * @brief  Command reset.
  * @param  CLI template
  * @retval CLI template
  */
static portBASE_TYPE prvResetCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ NVIC_SystemReset();
  return pdFALSE; }

// ---------------------------------------------------------------------------------------------------------------------------

static uint32_t valid_serial_speed[6] = { 4800, 9600, 19200, 38400, 57600, 115200 } ;

/**
  * @brief  Command set_cons_speed: sets console UART speed.
  * @param  CLI template
  * @retval CLI template
  */
static portBASE_TYPE prvSetConsSpeedCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ const char* param;
  BaseType_t  param_len;
  param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);
  int32_t speed=atol(param);
  int i;
  for(i=0; i<6; i++)
  { if(speed==valid_serial_speed[i]) break; }
  if(i==6) { sprintf(pcWriteBuffer, "Incorrect speed\r\n"); return pdFALSE; }
  SetOption(OPT_CONS_SPEED, &speed);
  sprintf(pcWriteBuffer, "New console speed set to %ld, please reset CPU.\r\n", speed);
  return pdFALSE; }

/**
  * @brief  Command set_gps_speed: sets GPS UART speed.
  * @param  CLI template
  * @retval CLI template
  */
static portBASE_TYPE prvSetGPSSpeedCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ const char* param;
  BaseType_t  param_len;
  param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);
  int32_t speed=atol(param);
  int i;
  for(i=0; i<6; i++)
  { if(speed==valid_serial_speed[i]) break; }
  if(i==6) { sprintf(pcWriteBuffer, "Incorrect speed\r\n"); return pdFALSE; }
  SetOption(OPT_GPS_SPEED, &speed);
  sprintf(pcWriteBuffer, "New GPS speed set to %ld, please reset CPU.\r\n", speed);
  return pdFALSE; }

/**
  * @brief  Command cons_speed: prints console UART speed.
  * @param  CLI template
  * @retval CLI template
  */
static portBASE_TYPE prvConsSpeedCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ uint32_t* cons_speed = (uint32_t*)GetOption(OPT_CONS_SPEED);
  if (cons_speed)
  { uint32_t speed = (*cons_speed);
    int i;
    for(i=0; i<6; i++)
    { if(speed == valid_serial_speed[i])
      { sprintf(pcWriteBuffer, "%ld\r\n", speed); break; }
    }
    if(i==6) sprintf(pcWriteBuffer, "Invalid console speed.\r\n");
  }
  else
  { sprintf(pcWriteBuffer, "Invalid parameter.\r\n"); }
  return pdFALSE; }

/**
  * @brief  Command gps_speed: prints GPS UART speed.
  * @param  CLI template
  * @retval CLI template
  */
static portBASE_TYPE prvGPSSpeedCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ uint32_t* gps_speed = (uint32_t*)GetOption(OPT_GPS_SPEED);
  if (gps_speed)
  { uint32_t speed = (*gps_speed);
    int i;
    for(i=0; i<6; i++)
    { if(speed == valid_serial_speed[i])
      { sprintf(pcWriteBuffer, "%ld\r\n", speed); break; }
    }
    if(i==6) sprintf(pcWriteBuffer, "Invalid GPS speed.\r\n");
  }
  else
  { sprintf(pcWriteBuffer, "Invalid parameter.\r\n"); }
  return pdFALSE; }

// ---------------------------------------------------------------------------------------------------------------------------

static int PrintAcftID(char *Output, uint32_t AcftID)         // Print and decode 32-bit aircraft ID
{ // static const char *AddrTypeName[4] = { "RND", "ICAO", "FLARM", "OGN" };
  uint32_t Address  =  AcftID     &0x00FFFFFF;                // Address: 24 bits
  uint8_t  AddrType = (AcftID>>24)&0x03;                      // AddrType: 2 bits => 0=random, 1=ICAO, 2=FLARM, 3=OGN
  uint8_t  AcftType = (AcftID>>26)&0x1F;                      // AcftType: 5 bits => 1=glider, 2=towing aircraft, 3=helicopter, 4=parachute, 5=drop-plane, 6=para-glider, 7=hang-glider, 8=powered airplane, 9=jet aircraft, 10=UFO, 11=baloon, 12=airship, 13=UAV/drone
  uint8_t  Private  = (AcftID>>31)&0x01;                      // Private:  1 bit  => 1=do not show on displays and maps
  return sprintf(Output, "Aicraft ID: %08lX = %c%02d:%d:%06lX\r\n", AcftID, Private?'p':' ', AcftType, AddrType, Address); }

/**
  * @brief  Command acft_id: prints aircraft identification.
  * @param  CLI template
  * @retval CLI template
  */
static portBASE_TYPE prvAcftIDCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ PrintAcftID(pcWriteBuffer, *(uint32_t *)GetOption(OPT_ACFT_ID) );
  return pdFALSE; }

/**
  * @brief  Command set_acft_id: sets the aircraft identification
  * @param  CLI template
  * @retval CLI template
  */
static portBASE_TYPE prvSetAcftIDCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ BaseType_t  param_len;
  const char *param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);
  if(param)
  { char *end;
    uint32_t ID=strtol(param, &end, 0x10);
    if(end && ((*end)==0))
    { SetOption(OPT_ACFT_ID, &ID);
      PrintAcftID(pcWriteBuffer, ID); }
    else
    { sprintf(pcWriteBuffer, "Invalid argument, must be 32-bit hex number\r\n"); }
  }
  return pdFALSE; }

// ---------------------------------------------------------------------------------------------------------------------------

int PrintTxPower(char *Output, float TxPower)
{ int Neg=0; if(TxPower<0) { Neg=1; TxPower=(-TxPower); }
  int Int = (int)floor(TxPower);
  int Frac = (int)floor((TxPower-Int)*10);
  if(Neg) Int=(-Int);
  return sprintf(Output, "TxPower = %+d.%d dBm\r\n", Int, Frac); }

static portBASE_TYPE prvTxPowerCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ PrintTxPower(pcWriteBuffer, *(float *)GetOption(OPT_TX_POWER) );
  return pdFALSE; }

static portBASE_TYPE prvSetTxPowerCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ BaseType_t  param_len;
  const char *param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);
  if(param)
  { char *end;
    float TxPower=strtof(param, &end);
    if(end && ((*end)==0) && (TxPower>=(-30.0)) && (TxPower<=14.0) )
    { SetOption(OPT_TX_POWER, &TxPower);
      PrintTxPower(pcWriteBuffer, TxPower); }
  }
  return pdFALSE; }

// ---------------------------------------------------------------------------------------------------------------------------

/**
  * @brief  Command gps_time: prints GPS UTC time.
  * @param  CLI template
  * @retval CLI template
  */
static portBASE_TYPE prvGPSTimeCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ uint32_t Time = GPS_GetPosition(NULL);
  sprintf(pcWriteBuffer,"GPS Time = %ldsec\r\n", Time);
  return pdFALSE; }

/**
  * @brief  Command gps_pos: prints GPS UTC time and position.
  * @param  CLI template
  * @retval CLI template
  */
static portBASE_TYPE prvGPSPosCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ GPS_GetPosition(pcWriteBuffer); strcat(pcWriteBuffer, "\r"); return pdFALSE; }

// ---------------------------------------------------------------------------------------------------------------------------

/**
  * @brief  Command spi1_send: send data over SPI1 bus.
  * @param  CLI template
  * @retval CLI template
  */
static portBASE_TYPE prvSPI1SendCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{
   const char* param;
   BaseType_t  param_len;
   int i;
   uint8_t     ctr;

   param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);

   /* check number of provided hex values */
   if (param_len%2 != 0)
   {
      sprintf(pcWriteBuffer, "Error: provide round bytes.\r\n");
      return pdFALSE;
   }

   /* check provided hex values */
   for (i=0; i<param_len; i++)
   {
       if (get_hex_val(param[i]) == -1)
       {
           sprintf(pcWriteBuffer, "Error: provide hex values only.\r\n");
           return pdFALSE;
       }
   }

   /* convert string to array */
   for (i=0; i<param_len; i=i+2)
   {
      SPI1_tx_data[i>>1] = get_hex_str_val(&param[i]);
   }

   SPI1_Send(SPI1_tx_data, SPI1_rx_data, param_len>>1);

   /* Print MISO output after transfer */
   ctr = 0;
   for (i=0 ; i < param_len>>1; i++)
   {
      ctr+= print_hex_val(SPI1_rx_data[i], &pcWriteBuffer[ctr]);
   }
   pcWriteBuffer[ctr++] = '\r'; pcWriteBuffer[ctr++] = '\n';
   pcWriteBuffer[ctr++] = '\0';

   return pdFALSE;
}

/**
  * @brief  Command sp1_pkt: send OGN packet.
  * @param  CLI template
  * @retval CLI template
  */
static portBASE_TYPE prvSP1SendPacketCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{
   const char*  param;
   BaseType_t   param_len;
   int i;
   task_message sp1_msg;

   param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);

   /* check number of provided hex values */
   if (param_len != 2*OGN_PKT_LEN)
   {
      sprintf(pcWriteBuffer, "Error: provide 26 bytes.\r\n");
      return pdFALSE;
   }

   /* check provided hex values */
   for (i=0; i<param_len; i++)
   {
       if (get_hex_val(param[i]) == -1)
       {
           sprintf(pcWriteBuffer, "Error: provide hex values only.\r\n");
           return pdFALSE;
       }
   }

   /* convert string to array */
   for (i=0; i<param_len; i=i+2)
   {
      OGN_packet[i>>1] = get_hex_str_val(&param[i]);
   }
   xQueueHandle* sp1_task_queue = Get_SP1Que();

   sp1_msg.msg_data   = (uint32_t)&OGN_packet;
   sp1_msg.msg_len    = OGN_PKT_LEN;
   sp1_msg.msg_opcode = SP1_SEND_OGN_PKT;
   sp1_msg.src_id     = CONSOLE_USART_SRC_ID;
   /* Send NMEA sentence to GPS task */
   xQueueSend(*sp1_task_queue, &sp1_msg, portMAX_DELAY);

   sprintf(pcWriteBuffer, "OGN packet sent.\r\n");
   return pdFALSE;
}

// ---------------------------------------------------------------------------------------------------------------------------

static const CLI_Command_Definition_t VerCommand           = { "ver",            "ver: version number and MCU ID\r\n",          prvVerCommand,           0 };
static const CLI_Command_Definition_t ResetCommand         = { "reset",          "reset: CPU reset\r\n",                        prvResetCommand,         0 };

static const CLI_Command_Definition_t ConsSpeedCommand     = { "cons_speed",     "cons_speed: console USART speed\r\n",         prvConsSpeedCommand,     0 };
static const CLI_Command_Definition_t SetConsSpeedCommand  = { "set_cons_speed", "set_cons_speed: set console USART speed\r\n", prvSetConsSpeedCommand,  1 };
static const CLI_Command_Definition_t GPSSpeedCommand      = { "gps_speed",      "gps_speed: GPS USART speed\r\n",              prvGPSSpeedCommand,      0 };
static const CLI_Command_Definition_t SetGPSSpeedCommand   = { "set_gps_speed",  "set_gps_speed: set GPS USART speed\r\n",      prvSetGPSSpeedCommand,   1 };

static const CLI_Command_Definition_t SPI1SendCommand      = { "spi1",           "spi1 hex_vals: send data over SPI1\r\n",      prvSPI1SendCommand,      1 };
static const CLI_Command_Definition_t SP1SendPacketCommand = { "sp1_pkt",        "sp1_pkt 26xhex: send OGN packet\r\n",         prvSP1SendPacketCommand, 1 };

static const CLI_Command_Definition_t GPSTimeCommand       = { "gps_time",       "gps_time: GPS UTC Time\r\n",                  prvGPSTimeCommand,       0 };
static const CLI_Command_Definition_t GPSPosCommand        = { "gps_pos",        "gps_pos: GPS Time & Position\r\n",            prvGPSPosCommand,        0 };

static const CLI_Command_Definition_t AcftIDCommand        = { "acft_id",        "acft_id: aircraft identification\r\n",        prvAcftIDCommand,        0 };
static const CLI_Command_Definition_t SetAcftIDCommand     = { "set_acft_id",    "set_acft_id: set aircraft ident.\r\n",        prvSetAcftIDCommand,     1 };

static const CLI_Command_Definition_t TxPowerCommand       = { "tx_power",       "tx_power: RF transmitter power [dBm]\r\n",    prvTxPowerCommand,       0 };
static const CLI_Command_Definition_t SetTxPowerCommand    = { "set_tx_power",   "set_tx_power: set transm. power [dBm].\r\n",  prvSetTxPowerCommand,    1 };


/**
  * @brief  Function registers all console commands.
  * @param  None
  * @retval None
  */
void RegisterCommands(void)
{
   /* The commands are displayed in help in the order provided here */
   FreeRTOS_CLIRegisterCommand(&VerCommand);
   FreeRTOS_CLIRegisterCommand(&ResetCommand);

   FreeRTOS_CLIRegisterCommand(&ConsSpeedCommand);
   FreeRTOS_CLIRegisterCommand(&SetConsSpeedCommand);

   FreeRTOS_CLIRegisterCommand(&GPSSpeedCommand);
   FreeRTOS_CLIRegisterCommand(&SetGPSSpeedCommand);

   FreeRTOS_CLIRegisterCommand(&SP1SendPacketCommand);
   FreeRTOS_CLIRegisterCommand(&SPI1SendCommand);

   FreeRTOS_CLIRegisterCommand(&GPSTimeCommand);
   FreeRTOS_CLIRegisterCommand(&GPSPosCommand);

   FreeRTOS_CLIRegisterCommand(&AcftIDCommand);
   FreeRTOS_CLIRegisterCommand(&SetAcftIDCommand);

   FreeRTOS_CLIRegisterCommand(&TxPowerCommand);
   FreeRTOS_CLIRegisterCommand(&SetTxPowerCommand);
}

// ---------------------------------------------------------------------------------------------------------------------------
