#include "ogn.h"

#include "gps.h"
#include <stm32l1xx.h>
#include <FreeRTOS.h>
#include <FreeRTOS_CLI.h>
#include <task.h>
#include <semphr.h>
#include "usart.h"
#include "messages.h"
#include "cir_buf.h"
#include "console.h"
#include "options.h"

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

static SemaphoreHandle_t xGpsPosMutex = 0;
static int PosPtr=0;
static OgnPosition Position[4]; // we keep the 3 most recent positions
// static OGN_Packet  Packet;

 uint32_t GPS_GetUnixTime(void)
{ xSemaphoreTake(xGpsPosMutex, portMAX_DELAY);
  int Ptr=PosPtr; uint32_t Time=0;
  if(Position[Ptr].isComplete()) Time=Position[Ptr].UnixTime;
  else
  { Ptr = (Ptr-1)&3;
    if(Position[Ptr].isComplete()) Time=Position[Ptr].UnixTime;
  }
  xSemaphoreGive(xGpsPosMutex);
  return Time; }

 void Handle_NMEA_String(const char* str, uint8_t len)
{ 
  xSemaphoreTake(xGpsPosMutex, portMAX_DELAY);
  if(Position[PosPtr].ReadNMEA(str)<=0) return;
  if(Position[PosPtr].isComplete())
  { if(Position[PosPtr].isValid())                          // new position is complete: but GPS lock might not be there yet
    { int RefPtr = (PosPtr+2)&3;
      if(Position[RefPtr].isValid())
      { Position[PosPtr].calcDifferences(Position[RefPtr]); // measure climb/turn rates
      }
    }
    PosPtr = (PosPtr+1)&3; Position[PosPtr].Clear(); }
  xSemaphoreGive(xGpsPosMutex);
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
   for(int Pos=0; Pos<4; Pos++)
     Position[Pos].Clear();
   PosPtr=0;

   xGpsPosMutex = xSemaphoreCreateMutex();

   task_message msg;

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
