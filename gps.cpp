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
#include "spirit1.h"

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
static OGN_Packet  Packet;

 uint32_t GPS_GetPosition(char *Output)
{ xSemaphoreTake(xGpsPosMutex, portMAX_DELAY);
  int Ptr=PosPtr; uint32_t Time=0; if(Output) Output[0]=0;
  if(Position[Ptr].isComplete())
  { Time=Position[Ptr].UnixTime; if(Output) Position[Ptr].PrintLine(Output); }
  else
  { Ptr = (Ptr-1)&3;
    if(Position[Ptr].isComplete())
    { Time=Position[Ptr].UnixTime; if(Output) Position[Ptr].PrintLine(Output); }
  }
  xSemaphoreGive(xGpsPosMutex);
  return Time; }

 void Handle_NMEA_String(const char* str, uint8_t len)
{ task_message sp1_msg;

  xSemaphoreTake(xGpsPosMutex, portMAX_DELAY);
  if(Position[PosPtr].ReadNMEA(str)>0)
  { if(Position[PosPtr].isComplete())
    { if(Position[PosPtr].isValid())                          // new position is complete: but GPS lock might not be there yet
      { int RefPtr = (PosPtr+2)&3;
        if(Position[RefPtr].isValid())
        { int Delta=Position[PosPtr].calcDifferences(Position[RefPtr]); // measure climb/turn rates
          if((Delta>0)&&(Delta<=5))
          { Packet.setAddress(0xE01234); Packet.setAddrType(3); Packet.calcAddrParity();
            Packet.setRelayCount(0);
            Position[PosPtr].Encode(Packet);
            Packet.setAcftType(0x1); Packet.clrPrivate();
            Packet.Encrypt();
            Packet.setFEC();
            sp1_msg.msg_data   = (uint32_t)&Packet.Header;
            sp1_msg.msg_len    = OGN_PKT_LEN;
            sp1_msg.msg_opcode = SP1_SEND_OGN_PKT;
            sp1_msg.src_id     = GPS_USART_SRC_ID;
            xQueueHandle* sp1_task_queue = Get_SP1Que();
            xQueueSend(*sp1_task_queue, &sp1_msg, portMAX_DELAY);
          }
        }
      }
      PosPtr = (PosPtr+1)&3; Position[PosPtr].Clear();
    }
  }
  xSemaphoreGive(xGpsPosMutex);
  return; }

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
   Packet.Clear();

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
