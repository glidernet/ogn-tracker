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


OGN_Parse_res_t OGN_Parse_NMEA(const char* str, uint8_t len)                   // process NMEA from the GPS
{ 
  OGN_Parse_res_t ret_value;
  xSemaphoreTake(xOgnPosMutex, portMAX_DELAY);
  int Delta; int PrevPtr;
  int Ret=Position[PosPtr].ReadNMEA(str);
  if(Ret<0)                                { ret_value = OGN_PARSE_BAD_NMEA;         goto Exit; }  // bad NMEA
  if(Ret==0)                               { ret_value = OGN_PARSE_NO_USEFUL_NMEA;   goto Exit; }  // no useful NMEA 
  if(!Position[PosPtr].isComplete())       { ret_value = OGN_PARSE_POS_NOT_COMPLETE; goto Exit; }  // position is not yet complete, but the NMEA was useful
  if(!Position[PosPtr].isValid())          { ret_value = OGN_PARSE_POS_NOT_VALID;    goto Exit; }  // position is complete, but not valid (no GPS fix)
  PrevPtr=(PosPtr+2)&3; Delta=0;                                   // current position is complete and valid: look two position earlier
  if(Position[PrevPtr].isValid())
  { Delta=Position[PosPtr].calcDifferences(Position[PrevPtr]); }
  else
  { PrevPtr=(PosPtr+3)&3;
    Delta=Position[PosPtr].calcDifferences(Position[PrevPtr]); }
  PosPtr=(PosPtr+1)&3; ret_value=Delta<=5? OGN_PARSE_POS_VALID_CURRENT:OGN_PARSE_POS_VALID_5SECS_AGO; // GPS lock: check age (if 5 seconds ago)
                                                                 
 Exit:
  xSemaphoreGive(xOgnPosMutex);
  return ret_value; }

uint32_t OGN_GetPosition(char *Output)                             // print into a string current position and other GPS data
{ xSemaphoreTake(xOgnPosMutex, portMAX_DELAY);
  int Ptr=PosPtr; if(Output) Output[0]=0;
  if(Position[Ptr].isComplete())
  { if(Output) Position[Ptr].PrintLine(Output); }
  else
  { Ptr = (Ptr+3)&3;
  if(Position[Ptr].isComplete())
  { if(Output) Position[Ptr].PrintLine(Output); }
  }
  uint32_t Time=Position[Ptr].getUnixTime();
  xSemaphoreGive(xOgnPosMutex);
  return Time; }

uint8_t* OGN_PreparePacket(void)                                   // Prepare OGN packet
{ xSemaphoreTake(xOgnPosMutex, portMAX_DELAY);
  int Ptr = (PosPtr+3)&3; uint8_t* ret_data = 0;                   // get the frame just before the currect pointer
  if(Position[Ptr].isValid())                                      // is it valid ? (GPS lock ?)
  { uint32_t Address  =  AcftID     &0x00FFFFFF;                   // split ID into elements
    uint8_t  AddrType = (AcftID>>24)&0x03;
    uint8_t  AcftType = (AcftID>>26)&0x1F;
    uint8_t  Private  = (AcftID>>31)&0x01;

    Packet.setAddress(Address); Packet.setAddrType(AddrType); Packet.clrMeteo(); Packet.calcAddrParity();
    Packet.clrEmergency(); Packet.clrEncrypted(); Packet.setRelayCount(0);
    Position[Ptr].Encode(Packet);                                   // encode position into the packet
    Packet.setAcftType(AcftType);                                   // set aircraft type
    if(Private) Packet.setPrivate();                                // set private/stealth flag
           else Packet.clrPrivate();
    Packet.Whiten();                                                // Whiten the position/speed data (not the header)
    Packet.setFEC();                                                // compute the parity checks
    ret_data = (uint8_t*)&Packet.Header; }                          // get pointer to packet bytes: works only with little-endian CPU
  xSemaphoreGive(xOgnPosMutex);
  return ret_data; }

