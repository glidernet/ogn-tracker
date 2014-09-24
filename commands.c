#include "commands.h"
#include <stm32l1xx.h>
#include <stdio.h>
#include <FreeRTOS.h>
#include <FreeRTOS_CLI.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <string.h>
#include "options.h"

/* -------- defines -------- */
/* -------- variables -------- */
/* -------- constants -------- */
static const char * const pcVersion = "0.0.1\r\n";
/* -------- functions -------- */

/**
  * @brief  Command ver: prints version string.
  * @param  CLI template
  * @retval CLI template
  */
static portBASE_TYPE prvVerCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{
   strcpy((char*)pcWriteBuffer, pcVersion);
   return pdFALSE;
}

/**
  * @brief  Command cons_speed: prints console UART speed.
  * @param  CLI template
  * @retval CLI template
  */
static portBASE_TYPE prvConsSpeedCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{
   uint32_t* cons_speed = (uint32_t*)GetOption(OPT_CONS_SPEED);

   if (cons_speed)
   {
      switch (*cons_speed)
      {
         case 4800:
         {
            sprintf(pcWriteBuffer, "4800\r\n");
            break;
         }
         case 115200:
         {
            sprintf(pcWriteBuffer, "115200\r\n");
            break;
         }
         default:
         {
            sprintf(pcWriteBuffer, "Invalid console speed.\r\n");
         }
      }
   }
   else
   {
       sprintf(pcWriteBuffer, "Invalid parameter.\r\n");
   }
   return pdFALSE;
}

/**
  * @brief  Command gps_speed: prints GPS UART speed.
  * @param  CLI template
  * @retval CLI template
  */
static portBASE_TYPE prvGPSSpeedCommand( char *pcWriteBuffer,
                             size_t xWriteBufferLen,
                             const char *pcCommandString )
{
   uint32_t* gps_speed = (uint32_t*)GetOption(OPT_GPS_SPEED);

   if (gps_speed)
   {
      switch (*gps_speed)
      {
         case 4800:
         {
            sprintf(pcWriteBuffer, "4800\r\n");
            break;
         }
         case 9600:
         {
            sprintf(pcWriteBuffer, "9600\r\n");
            break;
         }
         default:
         {
            sprintf(pcWriteBuffer, "Invalid GPS speed.\r\n");
         }
      }
   }
   else
   {
       sprintf(pcWriteBuffer, "Invalid parameter.\r\n");
   }
   return pdFALSE;
}

/* -------- additional command constants -------- */
static const CLI_Command_Definition_t ConsSpeedCommand =
{
    "cons_speed",
    "cons_speed: console USART speed\r\n",
    prvConsSpeedCommand,
    0
};

static const CLI_Command_Definition_t GPSSpeedCommand =
{
    "gps_speed",
    "gps_speed: GPS USART speed\r\n",
    prvGPSSpeedCommand,
    0
};

static const CLI_Command_Definition_t VerCommand =
{
    "ver",
    "ver: version number\r\n",
    prvVerCommand,
    0
};

/**
  * @brief  Function registers all console commands.
  * @param  None
  * @retval None
  */
void RegisterCommands(void)
{
   /* The commands are displayed in help in the order provided here */
   FreeRTOS_CLIRegisterCommand(&ConsSpeedCommand);
   FreeRTOS_CLIRegisterCommand(&GPSSpeedCommand);
   FreeRTOS_CLIRegisterCommand(&VerCommand);
}
