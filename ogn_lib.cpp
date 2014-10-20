#include <FreeRTOS.h>
#include <semphr.h>

#include "ogn_lib.h"
#include "ogn.h"

/* -------- defines -------- */
/* -------- variables -------- */
static int         PosPtr=0;        // round buffer pointer
static OgnPosition Position[4];     // keep a round-buffer of for positions
static OGN_Packet  Packet;          // encoded OGN packet
static uint32_t    AcftID;

static SemaphoreHandle_t xOgnPosMutex = 0;

/* -------- functions -------- */

uint8_t OGN_Init(void)
{ xOgnPosMutex = xSemaphoreCreateMutex();
  for(int Pos=0; Pos<4; Pos++)
    Position[Pos].Clear();
  PosPtr=0;
  Packet.Clear();
  AcftID = 0;
  return 0; }

void OGN_SetAcftID(uint32_t id)
{ AcftID = id; }

uint32_t OGN_GetPosition(char *Output)
{ xSemaphoreTake(xOgnPosMutex, portMAX_DELAY);
  int Ptr=PosPtr; if(Output) Output[0]=0;
  if(!Position[Ptr].isComplete())
  { Ptr = (Ptr-1)&3; if(!Position[Ptr].isComplete()) return 0; }
  if(Output) Position[Ptr].PrintLine(Output);
  uint32_t Time=Position[Ptr].UnixTime;
  xSemaphoreGive(xOgnPosMutex);
  return Time; }


int OGN_Parse_NMEA(const char* str, uint8_t len)                   // process NMEA from the GPS
{ xSemaphoreTake(xOgnPosMutex, portMAX_DELAY);
  int Delta; int PrevPtr;
  int Ret=Position[PosPtr].ReadNMEA(str); if(Ret<=0) goto Exit;    // return 0, when no useful NMEA (or negative when bad NMEA)
  if(!Position[PosPtr].isComplete())       { Ret=1;  goto Exit; }  // return 1, when position is not yet complete, but the NMEA was useful
  if(!Position[PosPtr].isValid())          { Ret=2;  goto Exit; }  // return 2, when position is complete, but not valid (no GPS fix)
  PrevPtr=(PosPtr+2)&3; Delta=0;
  if(Position[PrevPtr].isValid())
  { Delta=Position[PosPtr].calcDifferences(Position[PrevPtr]); }
  else
  { PrevPtr=(PosPtr+3)&3;
    Delta=Position[PosPtr].calcDifferences(Position[PrevPtr]); }
  PosPtr=(PosPtr+1)&3; Ret=Delta<=5?4:3;                           // return 3, when GPS lock but previos lock more than 5 seconds ago
                                                                   // return 4, when GPS lock and previous lock no more than 5 seconds ago.
 Exit:
  xSemaphoreGive(xOgnPosMutex);
  return Ret; }

uint8_t* OGN_PreparePacket(void)
{ xSemaphoreTake(xOgnPosMutex, portMAX_DELAY);
  int Ptr = (PosPtr-1)&3;
  uint8_t* ret_data = 0;
  if(Position[Ptr].isValid())
  { uint32_t Address  =  AcftID     &0x00FFFFFF;
    uint8_t  AddrType = (AcftID>>24)&0x03;
    uint8_t  AcftType = (AcftID>>26)&0x1F;
    uint8_t  Private  = (AcftID>>31)&0x01;

    Packet.setAddress(Address); Packet.setAddrType(AddrType); Packet.clrMeteo(); Packet.calcAddrParity();
    Packet.clrEmergency(); Packet.clrEncrypted(); Packet.setRelayCount(0);
    Position[Ptr].Encode(Packet);
    Packet.setAcftType(AcftType);
    if(Private) Packet.setPrivate();
           else Packet.clrPrivate();
    Packet.Encrypt();
    Packet.setFEC();            
    ret_data = (uint8_t*)&Packet.Header; }                          // works only with little-endian CPU
  xSemaphoreGive(xOgnPosMutex);
  return ret_data; }

