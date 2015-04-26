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

#include "ogn_lib.h"
#include "options.h"
#include "messages.h"
#include "spi.h"
#include "spirit1.h"
#include "gps.h"
#include "control.h"
#include "hpt_timer.h"

/* -------- defines -------- */
#define SPI_DATA_LEN 256
#define ACFT_ID_LEN    4

/* -------- variables -------- */
uint8_t SPI1_tx_data[SPI_DATA_LEN];
uint8_t SPI1_rx_data[SPI_DATA_LEN];

uint8_t OGN_packet[OGN_PKT_LEN];

/* -------- constants -------- */
static const char * const pcVersion = "0.4.2";
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
static portBASE_TYPE prvConsSpeedCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ const char* param;
  BaseType_t  param_len;
  param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);
  if(param)
  { int32_t speed=atol(param);
    int i;
    for(i=0; i<6; i++)
    { if(speed==valid_serial_speed[i]) break; }
    if(i==6) { sprintf(pcWriteBuffer, "Incorrect speed\r\n"); return pdFALSE; }
    SetOption(OPT_CONS_SPEED, &speed);
    sprintf(pcWriteBuffer, "Console UART speed: %ld bps (after a reset)\r\n", speed);
  } else { sprintf(pcWriteBuffer, "Console UART speed: %ld bps\r\n", *(uint32_t *)GetOption(OPT_CONS_SPEED)); }

  return pdFALSE; }

// ---------------------------------------------------------------------------------------------------------------------------

/**
  * @brief  Command set_gps_speed: sets GPS UART speed.
  * @param  CLI template
  * @retval CLI template
  */
static portBASE_TYPE prvGPSSpeedCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ const char* param;
  BaseType_t  param_len;
  param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);
  if(param)
  { int32_t speed=atol(param);
    int i;
    for(i=0; i<6; i++)
    { if(speed==valid_serial_speed[i]) break; }
    if(i==6) { sprintf(pcWriteBuffer, "Incorrect speed\r\n"); return pdFALSE; }
    SetOption(OPT_GPS_SPEED, &speed);
    sprintf(pcWriteBuffer, "GPS URAT speed: %ld bps (after a reset)\r\n", speed);
  } else { sprintf(pcWriteBuffer, "GPS UART speed: %ld bps\r\n", *(uint32_t *)GetOption(OPT_GPS_SPEED)); }
  return pdFALSE; }

// ---------------------------------------------------------------------------------------------------------------------------

static int PrintAcftID(char *Output, uint32_t AcftID)         // Print and decode 32-bit aircraft ID
{ static const char *AddrTypeName[4] = { "RND", "ICAO", "FLARM", "OGN" };
  uint32_t Address  =  AcftID     &0x00FFFFFF;                // Address: 24 bits
  uint8_t  AddrType = (AcftID>>24)&0x03;                      // AddrType: 2 bits => 0=random, 1=ICAO, 2=FLARM, 3=OGN
  uint8_t  AcftType = (AcftID>>26)&0x1F;                      // AcftType: 5 bits => 1=glider, 2=towing aircraft, 3=helicopter, 4=parachute, 5=drop-plane, 6=para-glider, 7=hang-glider, 8=powered airplane, 9=jet aircraft, 10=UFO, 11=baloon, 12=airship, 13=UAV/drone
  uint8_t  Private  = (AcftID>>31)&0x01;                      // Private:  1 bit  => 1=do not show on displays and maps
  return sprintf(Output, "Aicraft ID: %08lX = %c%02d:%s:%06lX\r\n", AcftID, Private?'p':' ', AcftType, AddrTypeName[AddrType], Address); }

/**
  * @brief  Command set_acft_id: sets the aircraft identification
  * @param  CLI template
  * @retval CLI template
  */
static portBASE_TYPE prvAcftIDCommand( char *pcWriteBuffer,
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
  } else { PrintAcftID(pcWriteBuffer, *(uint32_t *)GetOption(OPT_ACFT_ID) ); }
  return pdFALSE; }

// ---------------------------------------------------------------------------------------------------------------------------

static int PrintTxPower(char *Output, float TxPower)
{ int Neg=0; if(TxPower<0) { Neg=1; TxPower=(-TxPower); }
  int Int = (int)floor(TxPower);
  int Frac = (int)floor((TxPower-Int)*10);
  if(Neg) Int=(-Int);
  return sprintf(Output, "TxPower = %+d.%d dBm\r\n", Int, Frac); }

static int PrintMaxTxPower(char *Output, float TxPower)
{ int Neg=0; if(TxPower<0) { Neg=1; TxPower=(-TxPower); }
  int Int = (int)floor(TxPower);
  int Frac = (int)floor((TxPower-Int)*10);
  if(Neg) Int=(-Int);
  return sprintf(Output, "MaxTxPower = %+d.%d dBm\r\n", Int, Frac); }

static portBASE_TYPE prvTxPowerCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ 
    BaseType_t  param_len;
    const char *param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);
    if(param)
    { 
        char *end;
        float TxPower=strtof(param, &end);
        float min_power = SPIRIT1_LIB_MIN_POWER;
        float max_power = *(float *)GetOption(OPT_MAX_TX_PWR);
    
        if(end && ((*end)==0) && (TxPower>=min_power) && (TxPower<=max_power) )
        { 
            SetOption(OPT_TX_POWER, &TxPower);            
        } 
        else 
        {
            sprintf(pcWriteBuffer, "Invalid TX power, can't be greater than max. power \r\n");
            return pdFALSE; 
        }
    }
    
    PrintTxPower(pcWriteBuffer, *(float *)GetOption(OPT_TX_POWER) ); 
    return pdFALSE; 
}

static portBASE_TYPE prvMaxTxPowerCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ 
    BaseType_t  param_len;
    const char *param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);
  
    if (param)
    { 
        char *end;
        float TxPower=strtof(param, &end);
        if (end && ((*end)==0) && (TxPower>=(-30.0)))
        { 
            SetOption(OPT_MAX_TX_PWR, &TxPower);
        }
        else
        {   
            sprintf(pcWriteBuffer, "Invalid max. TX power\r\n");
            return pdFALSE;
        }
    }
    
    PrintMaxTxPower(pcWriteBuffer, *(float *)GetOption(OPT_MAX_TX_PWR) ); 
    
    return pdFALSE;
}

// ---------------------------------------------------------------------------------------------------------------------------

static portBASE_TYPE prvXtalCorrCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ BaseType_t  param_len;
  const char *param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);
  if(param)
  { char *end;
    int16_t XtalCorr=strtol(param, &end, 10);
    if(end && ((*end)==0) && (XtalCorr>=(-50)) && (XtalCorr<=50) )
    { SetOption(OPT_XTAL_CORR, &XtalCorr);
      sprintf(pcWriteBuffer, "XtalCorr = %+d ppm (after a reset)\r\n", (int)XtalCorr); }
  } else { sprintf(pcWriteBuffer, "XtalCorr = %+d ppm\r\n", *(int16_t *)GetOption(OPT_XTAL_CORR) ); }
  return pdFALSE; }

// ---------------------------------------------------------------------------------------------------------------------------

static portBASE_TYPE prvFreqOfsCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ BaseType_t  param_len;
  const char *param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);
  if(param)
  { char *end;
    int32_t FreqOfs=strtol(param, &end, 10);
    if(end && ((*end)==0) && (FreqOfs>=(-25000)) && (FreqOfs<=25000) )
    { SetOption(OPT_FREQ_OFS, &FreqOfs);
      sprintf(pcWriteBuffer, "FreqOfs = %+ld Hz (after a reset)\r\n", FreqOfs); }
  } else { sprintf(pcWriteBuffer, "FreqOfs = %+ld Hz\r\n", *(int32_t *)GetOption(OPT_FREQ_OFS) ); }
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

/**
  * @brief  Command gps_reset: sends GPS cold reset NMEA sentence.
  * @param  CLI template
  * @retval CLI template
  */
static portBASE_TYPE prvGPSResetCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ 
  GPS_Reset();
  sprintf(pcWriteBuffer, "GPS reset sentence sent\r\n");
  return pdFALSE; 
}

/**
  * @brief  Commands gps_on/off: control GPS operating mode.
  * @param  CLI template
  * @retval CLI template
  */
static portBASE_TYPE prvGPSOnCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{
  GPS_On();
  sprintf(pcWriteBuffer, "GPS on\r\n");
  return pdFALSE; 
}

/**
  * @brief  Commands gps_on/off: control GPS operating mode.
  * @param  CLI template
  * @retval CLI template
  */
static portBASE_TYPE prvGPSOffCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ 
  GPS_Off();  
  sprintf(pcWriteBuffer, "GPS off\r\n");
  return pdFALSE; 
}

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
   xQueueHandle* sp1_task_queue = Get_SP1Queue();

   sp1_msg.msg_data   = (uint32_t)&OGN_packet;
   sp1_msg.msg_len    = OGN_PKT_LEN;
   sp1_msg.msg_opcode = SP1_COPY_OGN_PKT;
   sp1_msg.src_id     = CONSOLE_USART_SRC_ID;
   /* Send packet data to Spirit1 task */
   xQueueSend(*sp1_task_queue, &sp1_msg, portMAX_DELAY);

   sp1_msg.msg_data   = 0;
   sp1_msg.msg_len    = 0;
   sp1_msg.msg_opcode = SP1_TX_PACKET;
   sp1_msg.src_id     = CONSOLE_USART_SRC_ID;
   /* TX packet */
   xQueueSend(*sp1_task_queue, &sp1_msg, portMAX_DELAY);
   
   sp1_msg.msg_data   = 0;
   sp1_msg.msg_len    = 0;
   sp1_msg.msg_opcode = SP1_COPY_OGN_PKT;
   sp1_msg.src_id     = CONSOLE_USART_SRC_ID;
   /* Clear packet data in Spirit1 task */
   xQueueSend(*sp1_task_queue, &sp1_msg, portMAX_DELAY);
   
   sprintf(pcWriteBuffer, "OGN packet sent.\r\n");
   return pdFALSE;
}

static portBASE_TYPE prvIWDGDisCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ 
    BaseType_t  param_len;
    uint8_t iwdg_dis = 0;
    const char* param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);
    if(param)
    {  
       if (!strcmp(param, "en"))
       {
           iwdg_dis = 0;
       }
       else if (!strcmp(param, "dis"))
       {
           iwdg_dis = 1;
       }
       SetOption(OPT_IWDG, &iwdg_dis);
    } 
    
    iwdg_dis = *(uint8_t *)GetOption(OPT_IWDG);
    if (!iwdg_dis)
    {
        sprintf(pcWriteBuffer, "IWDG enabled\r\n");
    }
    else
    {
        sprintf(pcWriteBuffer, "IWDG disabled\r\n");
    }
    return pdFALSE;
}
  
static portBASE_TYPE prvSetChannelCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{
    const char*  param;
    BaseType_t   param_len;
    int8_t       channel_number;
    
    param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);
    
    if (param)
    {
        channel_number = atoi(param);
        if ((channel_number >= 0) && (channel_number <= 6))
        {
            SetOption(OPT_CHANNEL, &channel_number);
        }
        else
        {
            sprintf(pcWriteBuffer, "Invalid channel number: %d.\r\n", channel_number);
            return pdFALSE;  
        }
    }
    /* get version of command */
    channel_number = *(uint8_t *)GetOption(OPT_CHANNEL);
    
    sprintf(pcWriteBuffer, "Channel set: %d.\r\n", channel_number);
    
    return pdFALSE;                         
}

static portBASE_TYPE prvOperModeCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{
    BaseType_t  param_len;
    uint8_t new_mode = MODE_OGN;
    const char* param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);
    
    /* optional set version of command */
    if(param)
    {  
       if (!strcmp(param, "ogn"))
       {
           new_mode = (uint8_t)MODE_OGN;
       }
       else if (!strcmp(param, "idle"))
       {
           new_mode = (uint8_t)MODE_IDLE;
       }
       else if (!strcmp(param, "cw"))
       {
           new_mode = (uint8_t)MODE_CW;
       }
       else if (!strcmp(param, "rx"))
       {
           new_mode = (uint8_t)MODE_RX;
       }
       else if (!strcmp(param, "jammer"))
       {
           new_mode = (uint8_t)MODE_JAMMER;
       }
       else
       {
            sprintf(pcWriteBuffer, "unsupported mode, supported: ogn, idle, cw, rx, jammer\r\n");
            return pdFALSE;            
       }
       SetOption(OPT_OPER_MODE, &new_mode);
    } 
    
    /* get version of command */
    new_mode = *(uint8_t *)GetOption(OPT_OPER_MODE);
    
    switch (new_mode)
    {
        case MODE_OGN:
            sprintf(pcWriteBuffer, "mode ogn enabled\r\n");
            break;
        case MODE_IDLE:   
            sprintf(pcWriteBuffer, "mode idle enabled\r\n");
            break;               
        case MODE_CW:   
            sprintf(pcWriteBuffer, "mode cw enabled\r\n");
            break;
        case MODE_RX:   
            sprintf(pcWriteBuffer, "mode rx enabled\r\n");
            break;
        case MODE_JAMMER:   
            sprintf(pcWriteBuffer, "mode jammer enabled (!)\r\n");
            break;           
        default:
            sprintf(pcWriteBuffer, "unsupported mode\r\n");
            break;
    }
    return pdFALSE;
}

static portBASE_TYPE prvMemStatCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{
    int heap_used = (int)configTOTAL_HEAP_SIZE - (int)xPortGetFreeHeapSize();
    sprintf(pcWriteBuffer,"Free RTOS heap used: %d/%d (%d%%)\r\n", heap_used, (int)configTOTAL_HEAP_SIZE, (100*heap_used)/(int)configTOTAL_HEAP_SIZE);
    return pdFALSE;
}

static portBASE_TYPE prvGPSDumpCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ 
    BaseType_t  param_len;
    uint8_t gpsdump = 0;
    
    const char* param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);
    if(param)
    {  
       if (!strcmp(param, "en"))
       {
           gpsdump = 1;
       }
       else if (!strcmp(param, "dis"))
       {
           gpsdump = 0;
       }
       SetOption(OPT_GPSDUMP, &gpsdump);
    } 
    
    gpsdump = *(uint8_t *)GetOption(OPT_GPSDUMP);
    if (gpsdump)
    {
        sprintf(pcWriteBuffer, "GPS dump enabled\r\n");
    }
    else
    {
        sprintf(pcWriteBuffer, "GPS dump disabled\r\n");
    }
    return pdFALSE;
}

static portBASE_TYPE prvGPSAlwONCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ 
    BaseType_t  param_len;
    uint8_t gps_alw_on = 0;
    
    const char* param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);
    if(param)
    {  
       if (!strcmp(param, "en"))
       {
           gps_alw_on = 1;
       }
       else if (!strcmp(param, "dis"))
       {
           gps_alw_on = 0;
       }
       SetOption(OPT_GPS_ALW_ON, &gps_alw_on);
    } 
    
    gps_alw_on = *(uint8_t *)GetOption(OPT_GPS_ALW_ON);
    if (gps_alw_on)
    {
        sprintf(pcWriteBuffer, "GPS always on - enabled\r\n");
    }
    else
    {
        sprintf(pcWriteBuffer, "GPS always on - disabled\r\n");
    }
    return pdFALSE;
}

static portBASE_TYPE prvBackupRegCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{
    BaseType_t  param_len;
    const char* param;
    uint32_t    reg_number;
    uint32_t    reg_value;
    
    /* Get register number */
    param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);
    
    if (param) reg_number = atoi(param);
    
    if (!((param)&&(IS_RTC_BKP(reg_number))))
    {
        sprintf(pcWriteBuffer, "Invalid register number, valid: 0-31.\r\n");
        return pdFALSE;
    }    
    /* Get register value if provided*/
    param = FreeRTOS_CLIGetParameter(pcCommandString, 2, &param_len);    
    if (param)
    {
        RTC_WriteBackupRegister(reg_number, atoi(param));
    }   
    reg_value = RTC_ReadBackupRegister(reg_number);
    sprintf(pcWriteBuffer, "Register %d: %d\r\n", (int)reg_number, (int)reg_value);
    
    return pdFALSE;
}

static portBASE_TYPE prvDebugGPSCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{
    GPS_Debug_on();
    sprintf(pcWriteBuffer,"GPS Debug enabled\r\n");
    return pdFALSE;
}

static portBASE_TYPE prvDebugHPTCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{
    static uint8_t debug_state = 1;
    sprintf(pcWriteBuffer,"HPT debug switched\r\n");
    HPT_Debug(debug_state);
    if (debug_state) debug_state = 0; else debug_state = 1;
    return pdFALSE;
}


static portBASE_TYPE prvGPSAntCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ 
    BaseType_t  param_len;
    uint8_t gps_ant = 0;
    
    const char* param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);
    if(param)
    {  
       if (!strcmp(param, "int"))
       {
           gps_ant = 0;
       }
       else if (!strcmp(param, "ext"))
       {
           gps_ant = 1;
       }
       SetOption(OPT_GPS_ANT, &gps_ant);
    } 
    
    gps_ant = *(uint8_t *)GetOption(OPT_GPS_ANT);
    if (gps_ant)
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_8);
        sprintf(pcWriteBuffer, "GPS external antenna used.\r\n");
    }
    else
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_8);
        sprintf(pcWriteBuffer, "GPS internal antenna used.\r\n");       
    }
    return pdFALSE;
}

static portBASE_TYPE prvVoltCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{
    __IO uint16_t ADCdata_vref = 0;
    __IO uint16_t ADCdata_vbat = 0;
    int vref = 0, vbat = 0;
    
    /* Internal STM32L reference voltage typical value */
    /* It could be calibrated if needed */
    const uint32_t vrefint = 1224; /* 1224 mV */
    
    ADC_Config(ADC_Channel_17); 
    ADCdata_vref = ADC_GetConversionValue(ADC1);
    ADC_DeInit(ADC1);
    
    ADC_Config(ADC_Channel_10); 
    ADCdata_vbat = ADC_GetConversionValue(ADC1);
    ADC_DeInit(ADC1);
    
    if (ADCdata_vref) vref = (4095*vrefint)/ADCdata_vref;
    if (ADCdata_vbat) vbat = (ADCdata_vbat*vrefint)/ADCdata_vref;
    /* VBat is 2 times lowered for measurement in HW */
    vbat *= 2;
    
    sprintf(pcWriteBuffer,"Vdd: %d[mV], Vbat: %d[mV]\r\n", vref, vbat);
    
    /* Return ADC to idle state */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);
    RCC_HSICmd(DISABLE);
    
    return pdFALSE;
}

static portBASE_TYPE prvJamRatioCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{ 
    BaseType_t  param_len;
    uint8_t jam_ratio = 0;
    
    const char* param = FreeRTOS_CLIGetParameter(pcCommandString, 1, &param_len);
    if(param)
    {  
       jam_ratio = atoi(param);
       SetOption(OPT_JAM_RATIO, &jam_ratio);
    } 
    
    jam_ratio = *(uint8_t *)GetOption(OPT_JAM_RATIO);
    sprintf(pcWriteBuffer, "Jamming ratio: %d.\r\n", jam_ratio);
    
    return pdFALSE;
}

// ---------------------------------------------------------------------------------------------------------------------------

static const CLI_Command_Definition_t VerCommand           = { "ver",            "ver: version number and MCU ID\r\n",           prvVerCommand,           0 };
static const CLI_Command_Definition_t ResetCommand         = { "reset",          "reset: CPU reset\r\n",                         prvResetCommand,         0 };

static const CLI_Command_Definition_t SPI1SendCommand      = { "spi1",           "spi1 hex_vals: send data over SPI1\r\n",       prvSPI1SendCommand,      1 };
static const CLI_Command_Definition_t SP1SendPacketCommand = { "sp1_pkt",        "sp1_pkt 26xhex: send OGN packet\r\n",          prvSP1SendPacketCommand, 1 };

static const CLI_Command_Definition_t GPSTimeCommand       = { "gps_time",       "gps_time: GPS UTC Time\r\n",                   prvGPSTimeCommand,   0 };
static const CLI_Command_Definition_t GPSPosCommand        = { "gps_pos",        "gps_pos: GPS Time & Position\r\n",             prvGPSPosCommand,    0 };

static const CLI_Command_Definition_t ConsSpeedCommand     = { "cons_speed",   "cons_speed: console USART speed\r\n",            prvConsSpeedCommand, -1 };
static const CLI_Command_Definition_t GPSSpeedCommand      = { "gps_speed",    "gps_speed: GPS USART speed\r\n",                 prvGPSSpeedCommand,  -1 };
static const CLI_Command_Definition_t GPSDumpCommand       = { "gpsdump",      "gpsdump en/dis: dump GPS output to console\r\n", prvGPSDumpCommand, -1 };
static const CLI_Command_Definition_t GPSResetCommand      = { "gps_reset",    "gps_reset: GPS cold reset.\r\n",                 prvGPSResetCommand, 0 };
static const CLI_Command_Definition_t GPSOnCommand         = { "gps_on",       "gps_on: turn GPS on.\r\n",                       prvGPSOnCommand, 0 };
static const CLI_Command_Definition_t GPSOffCommand        = { "gps_off",      "gps_off: turn GPS off.\r\n",                     prvGPSOffCommand, 0 };
static const CLI_Command_Definition_t GPSAlwONCommand      = { "gps_always_on","gps_always_on en/dis: disable GPS power control\r\n", prvGPSAlwONCommand, -1 };

static const CLI_Command_Definition_t AcftIDCommand        = { "acft_id",      "acft_id:  aircraft identification\r\n",          prvAcftIDCommand,    -1 };
static const CLI_Command_Definition_t TxPowerCommand       = { "tx_power",     "tx_power: transmitter power level [dBm].\r\n",   prvTxPowerCommand,   -1 };
static const CLI_Command_Definition_t XtalCorrCommand      = { "xtal_corr",    "xtal_corr: Crystal freq. correction [ppm].\r\n", prvXtalCorrCommand,  -1 };
static const CLI_Command_Definition_t FreqOfsCommand       = { "freq_ofs",     "freq_ofs:  RF frequency offset [Hz].\r\n",       prvFreqOfsCommand,   -1 };
static const CLI_Command_Definition_t IWDGDisCommand       = { "iwdg",         "iwdg en/dis: control ind. watchdog\r\n",         prvIWDGDisCommand,   -1 };
static const CLI_Command_Definition_t OperModeCommand      = { "mode",         "mode [ogn|idle|cw|rx|jammer]: set/check oper. mode\r\n", prvOperModeCommand,  -1 };
static const CLI_Command_Definition_t SetChannelCommand    = { "channel",      "channel 0-6: set/check operating channel\r\n",   prvSetChannelCommand, -1 };
static const CLI_Command_Definition_t MemStatCommand       = { "mem_stat",     "mem_stat: memory statistics\r\n",                prvMemStatCommand, 0 };
static const CLI_Command_Definition_t MaxTxPowerCommand    = { "max_tx_power", "max_tx_power: set max. measured power [dBm].\r\n", prvMaxTxPowerCommand,  -1 };
static const CLI_Command_Definition_t BackupRegCommand     = { "backup_reg",   "backup_reg reg [value].\r\n",                    prvBackupRegCommand,  -1 };
static const CLI_Command_Definition_t DebugGPSCommand      = { "debug_gps",    "debug_gps - enable GPS logging.\r\n",            prvDebugGPSCommand,  0 };
static const CLI_Command_Definition_t DebugHPTCommand      = { "debug_hpt",    "debug_hpt - enable HPT logging.\r\n",            prvDebugHPTCommand,  0 };
static const CLI_Command_Definition_t GPSAntCommand        = { "gps_ant",      "gps_ant [int|ext] - select GPS antenna.\r\n",    prvGPSAntCommand,  -1 };
static const CLI_Command_Definition_t VoltCommand          = { "volt",         "volt: show voltages.\r\n",                       prvVoltCommand, 0 };
static const CLI_Command_Definition_t JamRatioCommand      = { "jam_ratio",    "jam_ratio: [0-100].\r\n",                        prvJamRatioCommand, -1 };

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
   FreeRTOS_CLIRegisterCommand(&GPSSpeedCommand);

   FreeRTOS_CLIRegisterCommand(&SP1SendPacketCommand);
   FreeRTOS_CLIRegisterCommand(&SPI1SendCommand);

   FreeRTOS_CLIRegisterCommand(&GPSTimeCommand);
   FreeRTOS_CLIRegisterCommand(&GPSPosCommand);
   FreeRTOS_CLIRegisterCommand(&GPSDumpCommand);
   FreeRTOS_CLIRegisterCommand(&GPSResetCommand);
   FreeRTOS_CLIRegisterCommand(&GPSOnCommand);
   FreeRTOS_CLIRegisterCommand(&GPSOffCommand);
   FreeRTOS_CLIRegisterCommand(&GPSAlwONCommand);

   FreeRTOS_CLIRegisterCommand(&AcftIDCommand);
   FreeRTOS_CLIRegisterCommand(&TxPowerCommand);
   FreeRTOS_CLIRegisterCommand(&MaxTxPowerCommand);
   FreeRTOS_CLIRegisterCommand(&XtalCorrCommand);
   FreeRTOS_CLIRegisterCommand(&FreqOfsCommand);
   FreeRTOS_CLIRegisterCommand(&IWDGDisCommand);
   FreeRTOS_CLIRegisterCommand(&SetChannelCommand);
   FreeRTOS_CLIRegisterCommand(&OperModeCommand);
   FreeRTOS_CLIRegisterCommand(&MemStatCommand);
   FreeRTOS_CLIRegisterCommand(&BackupRegCommand);
   FreeRTOS_CLIRegisterCommand(&DebugGPSCommand);
   FreeRTOS_CLIRegisterCommand(&DebugHPTCommand);
   FreeRTOS_CLIRegisterCommand(&GPSAntCommand);
   FreeRTOS_CLIRegisterCommand(&VoltCommand);
   FreeRTOS_CLIRegisterCommand(&JamRatioCommand);
   
}

// ---------------------------------------------------------------------------------------------------------------------------
