#include <stdio.h>
#include <string.h>
#include <stm32l1xx.h>
#include <FreeRTOS.h>
#include <FreeRTOS_CLI.h>
#include <task.h>
#include <semphr.h>

#include "gps.h"

#include "usart.h"
#include "messages.h"
#include "cir_buf.h"
#include "console.h"
#include "options.h"
#include "spirit1.h"
#include "ogn_lib.h"

// #define GPS_DEBUG
/* -------- constants -------- */
/* http://support.maestro-wireless.com/knowledgebase.php?article=6 */
static const char * const pcResetNMEA = "$PSRF101,-2686727,-4304282,3851642,75000,95629,1684,12,4*24\r\n";
/* -------- defines ---------- */
/* -------- variables -------- */
/* circular buffer for NMEA messages */
cir_buf_str* nmea_buffer;
xQueueHandle gps_que;


/* -------- functions -------- */
/**
* @brief  Functions parses received NMEA string.
* @param  string address, string length
* @retval None
*/

uint32_t GPS_GetPosition(char *Output) { return OGN_GetPosition(Output); }

void Handle_NMEA_String(const char* str, uint8_t len)
{
#ifdef GPS_DEBUG
  static char DebugStr[120];
  int Ret=OGN_Parse_NMEA(str, len);
  sprintf(DebugStr, "NMEA:%6.6s[%2d] => %d\r\n", str, len, Ret);
  Console_Send(DebugStr, 0);
#else
  OGN_Parse_NMEA(str, len);
#endif
  return; }

/**
* @brief  GPS Cold Reset.
* @param  None
* @retval None
*/

void GPS_Reset(void)
{
    USART3_Send((uint8_t*)pcResetNMEA, strlen(pcResetNMEA));
}

/**
* @brief  Configures the GPS Task Peripherals.
* @param  None
* @retval None
*/

void GPS_Config(void)
{
   uint32_t* gps_speed = (uint32_t*)GetOption(OPT_GPS_SPEED);
   if (gps_speed)
   {
      USART3_Config(*gps_speed);
   } 
}

/**
* @brief  GPS Task.
* @param  None
* @retval None
*/
void vTaskGPS(void* pvParameters)
{
   task_message msg;

   OGN_Init();
   OGN_SetAcftID(*(uint32_t*)GetOption(OPT_ACFT_ID));
 
   /* Allocate data buffer */
   nmea_buffer = init_cir_buf(CIR_BUF_NMEA);
   /* Send cir. buf. handle for USART3 driver & console */
   USART3_SetBuf(nmea_buffer);
   Console_SetNMEABuf(nmea_buffer);

   /* Create queue for GPS task messages received */
   gps_que = xQueueCreate(10, sizeof(task_message));
   /* Inform USART driver about receiving queue */
   USART3_SetQue(&gps_que);
   /* Inform Console about receiving queue */
   Console_SetGPSQue(&gps_que);
   /* Enable USART when everything is set */
   USART_Cmd(USART3, ENABLE);

   for(;;)
   {
      xQueueReceive(gps_que, &msg, portMAX_DELAY);
      switch (msg.src_id)
      {
         case GPS_USART_SRC_ID:
         {
            /* Received NMEA sentence from real GPS */
            if (*(uint8_t*)GetOption(OPT_GPSDUMP))
            {
                /* Send received NMEA sentence to console (without blocking) */
                Console_Send((char*)msg.msg_data, 0);
            }
            Handle_NMEA_String((char*)msg.msg_data, msg.msg_len);
            break;
         }
         case CONSOLE_USART_SRC_ID:
         {
            /* Received NMEA sentence from console */
            Handle_NMEA_String((char*)msg.msg_data, msg.msg_len);
            break;
         }
         default:
         {
            break;
         }
      }
   }
}
