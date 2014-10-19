#include "ogn_lib.h"
#include "ogn.h"

/* -------- defines -------- */
/* -------- variables -------- */
static int PosPtr=0;
static OgnPosition Position[4]; // we keep the 3 most recent positions
static OGN_Packet Packet;
static uint32_t AcftID;

/* -------- functions -------- */

uint8_t OGN_Init(void)
{
    for(int Pos=0; Pos<4; Pos++) Position[Pos].Clear();
    PosPtr=0;
    Packet.Clear();
    AcftID = 0;
    return 0;
}

void OGN_SetAcftID(uint32_t id)
{
    AcftID = id;
}

uint32_t OGN_GetPosition(char *Output)
{ 
  int Ptr=PosPtr; uint32_t Time=0; if(Output) Output[0]=0;
  if(Position[Ptr].isComplete())
  { Time=Position[Ptr].UnixTime; if(Output) Position[Ptr].PrintLine(Output); }
  else
  { Ptr = (Ptr-1)&3;
    if(Position[Ptr].isComplete())
    { Time=Position[Ptr].UnixTime; if(Output) Position[Ptr].PrintLine(Output); }
  }
  return Time; 
}

void OGN_Parse_NMEA(const char* str, uint8_t len)
{   
    if(Position[PosPtr].ReadNMEA(str)>0)
    {   
        if(Position[PosPtr].isComplete())
        { 
            if(Position[PosPtr].isValid())  // new position is complete: but GPS lock might not be there yet  
            {               
                PosPtr = (PosPtr+1)&3; //Position[PosPtr].Clear();                          
            }
        }
    }
}

uint8_t* OGN_PreparePacket(void)
{ 
    int RefPtr = (PosPtr+2)&3;
    uint8_t* ret_data = NULL;
    
    if(Position[RefPtr].isValid())
    { 
        //int Delta=Position[PosPtr].calcDifferences(Position[RefPtr]); // measure climb/turn rates
        //if((Delta>0)&&(Delta<=5))
        { 
            uint32_t Address  =  AcftID     &0x00FFFFFF;
            uint8_t  AddrType = (AcftID>>24)&0x03;
            uint8_t  AcftType = (AcftID>>26)&0x1F;
            uint8_t  Private  = (AcftID>>31)&0x01;

            Packet.setAddress(Address); Packet.setAddrType(AddrType); Packet.calcAddrParity();
            Packet.setRelayCount(0);

            Position[PosPtr].Encode(Packet);
            Packet.setAcftType(AcftType);
            if(Private) Packet.setPrivate();
                   else Packet.clrPrivate();

            Packet.Encrypt();
            Packet.setFEC();
            
            ret_data = (uint8_t*)&Packet.Header;
        }
    }
    return ret_data;
}