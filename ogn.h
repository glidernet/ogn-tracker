#ifndef __OGN_H__
#define __OGN_H__

#include <stdio.h>

#include <string.h>
#include <stdint.h>
#include <time.h>

#include <math.h>

#include "bitcount.h"
#include "nmea.h"

#include "ldpc.h"

class OGN_Packet          // Packet structure for the OGN tracker
{ public:

   uint32_t Header;       // address:24, spare:2, flags: ICAO, encrypt, relay, emergency, parity

   uint32_t Position[4];  // 0: FixQual:2, Time:6, Lat:24
                          // 1: FixMode:1, Baro:1, DOP:6,  Lon:24
                          // 2: TurnRate:8, Speed:10, Alt: 14
                          // 3: Temp:8, Type:4, Private:1 Climb:9, Heading:10

   uint32_t FEC[2];       // Gallager code: 48 check bits for 160 user bits

   uint32_t TEA_Key[4];   // TEA key to encrypt the position - the key is open,
                          // the encryption is for better statistical bit distributions and error correction

   int sendBytes(uint8_t *Packet) const             // make the bytes to be sent out in the RF packet
   { int ByteIdx=0; const uint32_t *WordPtr=&Header;
     for(int WordIdx=0; WordIdx<7; WordIdx++)
     { uint32_t Word=WordPtr[WordIdx];
       for(int Idx=0; Idx<4; Idx++)
       { if(ByteIdx>=26) break;
         Packet[ByteIdx++]=Word; Word>>=8; }
     }
     return 26; }
   int recvBytes(uint8_t *Packet)                  // get bytes from an RF packet and make the OGN_Packet
   { int ByteIdx=0; uint32_t *WordPtr=&Header;
     for(int WordIdx=0; WordIdx<7; WordIdx++)
     { uint32_t Word=0;
       for(int Idx=0; Idx<4; Idx++)
       { if(ByteIdx>=26) break;
         Word |= (uint32_t)(Packet[ByteIdx++])<<(Idx*8); }
       WordPtr[WordIdx]=Word;
     }
     return 26; }

   void Dump(void) const
   { printf("%08lX: %08lX %08lX %08lX %08lX [%08lX %04lX] (%d)\n",
             Header, Position[0], Position[1], Position[2], Position[3], FEC[0], FEC[1], checkFEC() ); }

   void DumpBytes(void) const
   { uint8_t Data[26]; sendBytes(Data);
     for(int Idx=0; Idx<26; Idx++)
     { printf(" %02X", Data[Idx]); }
     printf(" (%d)\n", LDPC_Check(Data)); }

   void Print(void) const
   { printf("%06lX%c R%c %c%01lX %c", getAddress(), isICAO()?'I':' ', '0'+getRelayed(), isPrivate()?'p':' ', getAcftType(), isEmergency()?'E':' ');
     printf("%ld/%ldD/%4.1f %02ldsec: [%+10.6f, %+10.6f]deg %ldm %3.1fkt %05.1fdeg %+4.1fm/s %+4.1fdeg/s\n",
            getFixQuality(), getFixMode()+2, 0.1*(10+DecodeDOP()), getTime(),
            0.0001/60*DecodeLatitude(), 0.0001/60*DecodeLongitude(), DecodeAltitude(),
            0.2*DecodeSpeed(), 0.1*DecodeHeading(), 0.1*DecodeClimbRate(), 0.1*DecodeTurnRate() ); }

   OGN_Packet() { Clear(); }
   void Clear(void) { Header=0; Position[0]=0; Position[1]=0; Position[2]=0; Position[3]=0; setKey(); }

   void setKey(const char *Key = "OGN DEFAULT KEY")
   { for(int KeyIdx=0; KeyIdx<4; KeyIdx++)
     { uint32_t Word=0; 
       for(int ByteIdx=0; ByteIdx<4; ByteIdx++)
       { uint8_t Byte=(*Key); Word = (Word<<8) | Byte;
         if(Byte) Key++; }
       TEA_Key[KeyIdx]=Word; }
   }

   void setFEC(void)            { LDPC_Encode(&Header, FEC); }       // calculate the 48-bit parity check
   void setFEC(const uint32_t ParityGen[48][5]) { LDPC_Encode(&Header, FEC, ParityGen); }
   int checkFEC(void)    const  { return LDPC_Check(&Header, FEC); } // returns number of parity checks that fail (0 => no errors, all fine)
   void Encrypt(void) { TEA_Encrypt(Position, TEA_Key, 4); TEA_Encrypt(Position+2, TEA_Key, 8); } // encrypt (hash) the position
   void Decrypt(void) { TEA_Decrypt(Position, TEA_Key, 4); TEA_Decrypt(Position+2, TEA_Key, 8); } // decrypt (de-hash) the position

   int BitErr(OGN_Packet &RefPacket) const // return number of different data bits between this Packet and RefPacket
   { return Count1s(Header^RefPacket.Header)
           +Count1s(Position[0]^RefPacket.Position[0])
           +Count1s(Position[1]^RefPacket.Position[1])
           +Count1s(Position[2]^RefPacket.Position[2])
           +Count1s(Position[3]^RefPacket.Position[3])
           +Count1s(FEC[0]^RefPacket.FEC[0])
           +Count1s((FEC[1]^RefPacket.FEC[1])&0xFFFF); }

   bool isEmergency(void)   const { return Header &  0x80000000; } // emergency declared or detected (high-g shock ?)
   void setEmergency(void)        {        Header |= 0x80000000; }
   void clrEmergency(void)        {        Header &= 0x7FFFFFFF; }

   bool  isEncrypted(void) const  { return Header &  0x40000000; } // position can be encrypted with a public key (competitions, etc.)
   void setEncrypted(void)        {        Header |= 0x40000000; } // when in Emergency it must not be encrypted
   void clrEncrypted(void)        {        Header &= 0xBFFFFFFF; }

   uint8_t getRelayCount(void) const { return (Header>>28)&0x03; } // how many time the packet has been relayed
   void    setRelayCount(uint8_t Count) { Header = (Header&0xCFFFFFFF) | ((uint32_t)(Count&0x03)<<28); }

   bool goodAddrParity(void) const  { return ((Count1s(Header&0x0FFFFFFF)&1)==0); }  // Address parity should be EVEN
   void calcAddrParity(void)        { if(!goodAddrParity()) Header ^= 0x08000000; }  // if not correct parity, flip the parity bit

   uint8_t getRelayed(void) const { return (Header>>25)&0x03; }    // 0=direct, 1=relayed OGN, 2=relayed FLARM, 3=reception report
   void    setRelayed(uint8_t Type) { Header = (Header&0xF9FFFFFF) | ((uint32_t)(Type&0x03)<<25); }

//   bool  isRelayed(void)   const  { return Header &  0x04000000; } // has been heard and is being relayed 
//   void setRelayed(void)          {        Header |= 0x04000000; } // to track aircrafts at low altitudes or in difficult terrains
//   void clrRelayed(void)          {        Header &= 0xFBFFFFFF; }

//   bool  isRelayFLARM(void) const { return Header &  0x02000000; } // is a relayed FLARM position
//   void setRelayFLARM(void)       {        Header |= 0x02000000; } // if tracker is receives position from a FLARM unit
//   void clrRelayFLARM(void)       {        Header &= 0xFDFFFFFF; }

   bool  isICAO(void) const       { return Header &  0x01000000; } // address is ICAO assigned
   void setICAO(void)             {        Header |= 0x01000000; }
   void clrICAO(void)             {        Header &= 0xFEFFFFFF; }

   uint32_t getAddress(void) const { return Header&0x00FFFFFF; }
   void setAddress(uint32_t Address) { Header = (Header&0xFF000000) | (Address&0x00FFFFFF); }


   bool  isPrivate(void) const  { return Position[3] &  0x00080000; } // position not to be displayed on public webpages
   void setPrivate(void)        {        Position[3] |= 0x00080000; }
   void clrPrivate(void)        {        Position[3] &= 0xFFF7FFFF; }

   uint32_t getAcftType(void) const { return (Position[3]>>20)&0x0F; }
   void     setAcftType(uint32_t Type) { Position[3] = (Position[3]&0xFF0FFFFF) | ((Type&0x0F)<<20); }

   uint32_t getTime(void) const { return (Position[0]>>24)&0x3F; }              // 6 lower bits of the UnitTime or the second counter ?
   void     setTime(uint32_t Time) { Position[0] = (Position[0]&0xC0FFFFFF) | ((Time&0x3F)<<24); }

   uint32_t getFixMode(void) const { return (Position[1]>>31)&0x01; }           // 0 = 2-D, 1 = 3-D
   void     setFixMode(uint32_t Mode) { Position[1] = (Position[1]&0x7FFFFFFF) | ((Mode&0x01)<<31); }

   bool  isBaro(void) const  { return Position[1] &  0x40000000; } // climb and alitude are takens with the barometer
   void setBaro(void)        {        Position[1] |= 0x40080000; } // after processing and calibration with the GPS
   void clrBaro(void)        {        Position[1] &= 0xBFFFFFFF; }

   uint32_t getFixQuality(void) const { return (Position[0]>>30)&0x03; }        // 0 = no fix, 1 = GPS, 2 = diff. GPS, 3 = other
   void     setFixQuality(uint32_t Qual) { Position[0] = (Position[0]&0x3FFFFFFF) | ((Qual&0x03)<<30); }

   int32_t static RoundDiv(int32_t Value, int32_t Div)
   { return Value>0 ? (Value+Div/2)/Div : (Value-Div/2)/Div; }

   void EncodeLatitude(int32_t Latitude)                             // encode Latitude: units are 0.0001/60 degrees, internally it is scaled down by 8, thus 1.5 meter resolution
   { Latitude>>=3; Position[0] = (Position[0]&0xFF000000) | (Latitude&0x00FFFFFF); }

   int32_t DecodeLatitude(void) const
   { int32_t Latitude=Position[0]&0x00FFFFFF;
     if(Latitude&0x00800000) Latitude|=0xFF000000;
     Latitude = (Latitude<<3)+4; return Latitude; }

   void EncodeLongitude(int32_t Longitude)                             // encode Longitude: units are 0.0001/60 degrees, internally it is scaled by 16, thus 0(pole)-3(equator) meter resolution
   { Longitude>>=4; Position[1] = (Position[1]&0xFF000000) | (Longitude&0x00FFFFFF); }

   int32_t DecodeLongitude(void) const
   { int32_t Longitude=Position[1]&0x00FFFFFF;
     if(Longitude&0x00800000) Longitude|=0xFF000000;
     Longitude = (Longitude<<4)+8; return Longitude; }

   void EncodeAltitude(int32_t Altitude)        // encode altitude in meters
   {      if(Altitude<0)      Altitude=0;
     else if(Altitude<0x1000) { }
     else if(Altitude<0x3000) Altitude = 0x1000 | ((Altitude-0x1000)>>1);
     else if(Altitude<0x7000) Altitude = 0x2000 | ((Altitude-0x3000)>>2);
     else if(Altitude<0xF000) Altitude = 0x3000 | ((Altitude-0x7000)>>3);
     else                     Altitude = 0x3FFF;
     Position[2] = (Position[2]&0xFFFFC000) | (Altitude&0x3FFF); }

   int32_t DecodeAltitude(void) const            // return Altitude in meters
   { int32_t Altitude =  Position[2]     &0x0FFF;
     int32_t Range    = (Position[2]>>12)&0x0003;
     if(Range==0) return         Altitude;       // 0000..0FFF
     if(Range==1) return 0x1001+(Altitude<<1);   // 1000..2FFE
     if(Range==2) return 0x3002+(Altitude<<2);   // 3000..6FFC
                  return 0x7004+(Altitude<<3); } // 7000..EFF8 => max. altitude: 61432 meters

   void EncodeDOP(int32_t DOP)
   {      if(DOP<0)    DOP=0;
     else if(DOP<0x10) { }
     else if(DOP<0x30) DOP = 0x10 | ((DOP-0x10)>>1);
     else if(DOP<0x70) DOP = 0x20 | ((DOP-0x30)>>2);
     else if(DOP<0xF0) DOP = 0x30 | ((DOP-0x70)>>3);
     else              DOP = 0x3F;
     Position[1] = (Position[1]&0xC0FFFFFF) | (DOP<<24); }

   int32_t DecodeDOP(void) const
   { int32_t DOP   =  (Position[1]>>24)&0x0F;
     int32_t Range =  (Position[1]>>28)&0x03;
     if(Range==0) return       DOP;              // 00..0F
     if(Range==1) return 0x11+(DOP<<1);          // 10..2E
     if(Range==2) return 0x31+(DOP<<2);          // 30..6C
                  return 0x74+(DOP<<4); }        // 70..E8 => max. DOP = 232*0.1=23.2

   void EncodeSpeed(int32_t Speed)            // speed in 0.2 knots
   {      if(Speed<0)     Speed=0;
     else if(Speed<0x100) { }
     else if(Speed<0x300) Speed = 0x100 | ((Speed-0x100)>>1);
     else if(Speed<0x700) Speed = 0x200 | ((Speed-0x300)>>2);
     else if(Speed<0xF00) Speed = 0x300 | ((Speed-0x700)>>3);
     else                 Speed = 0x3FF;
     Position[2] = (Position[2]&0xFF003FFF) | (Speed<<14); }

   int32_t DecodeSpeed(void) const           // return speed in 0.2 knots units (0.2 knots is about 0.1 m/s)
   { int32_t Speed = (Position[2]>>14)&0x00FF;
     int32_t Range = (Position[2]>>22)&0x0003;
     if(Range==0) return Speed;              // 000..0FF
     if(Range==1) return 0x101+(Speed<<1);   // 100..2FE
     if(Range==2) return 0x302+(Speed<<2);   // 300..6FC
                  return 0x704+(Speed<<3); } // 700..EF8 => max. speed: 3832*0.2 = 766 knots

   void EncodeTurnRate(int32_t Turn)
   { int32_t Sign=0; if(Turn<0) { Turn=(-Turn); Sign=0x80; }
          if(Turn<0x020) { }
     else if(Turn<0x060) Turn = 0x020 | ((Turn-0x020)>>1);
     else if(Turn<0x0E0) Turn = 0x040 | ((Turn-0x060)>>2);
     else if(Turn<0x1E0) Turn = 0x060 | ((Turn-0x0E0)>>3);
     else                Turn = 0x07F;
     Turn |= Sign;
     Position[2] = (Position[2]&0x00FFFFFF) | (Turn<<24); }

   int32_t DecodeTurnRate(void) const
   { int32_t Sign =(Position[2]>>31)&0x01;
     int32_t Range=(Position[2]>>29)&0x03;
     int32_t Turn =(Position[2]>>24)&0x1F;
          if(Range==0) { }                          // 000..01F
     else if(Range==1) { Turn = 0x021+(Turn<<1); }  // 020..05E
     else if(Range==2) { Turn = 0x062+(Turn<<2); }  // 060..0DC
     else              { Turn = 0x0E4+(Turn<<3); }  // 0E0..1D8 => max. turn rate = +/- 472*0.1 = +/- 47.2 deg/s
     return Sign ? -Turn:Turn; }

   int32_t DecodeHeading(void) const         // return Heading in 0.1 degree units
   { int32_t Heading = Position[3]&0x3FF;
     return (Heading*3600+512)>>10; }

   void EncodeHeading(int32_t Heading)
   { Heading = ((Heading<<10)+180)/3600;
     Position[3] = (Position[3]&0xFFFFFC00) | ((Heading&0x3FF)); }

   void EncodeClimbRate(int32_t Climb)
   { int32_t Sign=0; if(Climb<0) { Climb=(-Climb); Sign=0x100; }
          if(Climb<0x040) { }
     else if(Climb<0x0C0) Climb = 0x040 | ((Climb-0x040)>>1);
     else if(Climb<0x1C0) Climb = 0x080 | ((Climb-0x0C0)>>2);
     else if(Climb<0x3C0) Climb = 0x0C0 | ((Climb-0x1C0)>>3);
     else                 Climb = 0x0FF;
     Climb |= Sign;
     Position[3] = (Position[3]&0xFFF803FF) | (Climb<<10); }

   int32_t DecodeClimbRate(void) const
   { int32_t Sign =(Position[3]>>18)&0x01;
     int32_t Range=(Position[3]>>16)&0x03;
     int32_t Climb=(Position[3]>>10)&0x3F;
          if(Range==0) { }                            // 000..03F
     else if(Range==1) { Climb = 0x041+(Climb<<1); }  // 040..0BE
     else if(Range==2) { Climb = 0x0C2+(Climb<<2); }  // 0C0..1BC
     else              { Climb = 0x1C4+(Climb<<3); }  // 1C0..3B8 => max. climb rate = +/- 952*0.1 = +/- 95.2 m/s
     return Sign ? -Climb:Climb; }

   void EncodeTemperature(int32_t Temp)
   { int32_t Sign=0; if(Temp<0) { Temp=(-Temp); Sign=0x80; }
          if(Temp<0x020) { }
     else if(Temp<0x060) Temp = 0x020 | ((Temp-0x020)>>1);
     else if(Temp<0x0E0) Temp = 0x040 | ((Temp-0x060)>>2);
     else if(Temp<0x1E0) Temp = 0x060 | ((Temp-0x0E0)>>3);
     else                Temp = 0x07F;
     Temp |= Sign;
     Position[3] = (Position[3]&0x00FFFFFF) | (Temp<<24); }

   int32_t DecodeTemperature(void) const
   { int32_t Sign =(Position[3]>>31)&0x01;
     int32_t Range=(Position[3]>>29)&0x03;
     int32_t Temp =(Position[3]>>24)&0x1F;
          if(Range==0) { }                          // 000..01F
     else if(Range==1) { Temp = 0x021+(Temp<<1); }  // 020..05E
     else if(Range==2) { Temp = 0x062+(Temp<<2); }  // 060..0DC
     else              { Temp = 0x0E4+(Temp<<3); }  // 0E0..1D8 => max. temperature = +/- 472*0.5 = +/- 236 degC
     return Sign ? -Temp:Temp; }

   static void TEA_Encrypt (uint32_t* Data, const uint32_t Key[4], int Loops=8)
   { uint32_t v0=Data[0], v1=Data[1];                     // set up
     const uint32_t delta=0x9e3779b9; uint32_t sum=0;     // a key schedule constant
     uint32_t k0=Key[0], k1=Key[1], k2=Key[2], k3=Key[3]; // cache key
     for (int i=0; i < Loops; i++)                        // basic cycle start
     { sum += delta;
       v0 += ((v1<<4) + k0) ^ (v1 + sum) ^ ((v1>>5) + k1);
       v1 += ((v0<<4) + k2) ^ (v0 + sum) ^ ((v0>>5) + k3); }  // end cycle
     Data[0]=v0; Data[1]=v1;
   }

   void TEA_Decrypt (uint32_t* Data, const uint32_t Key[4], int Loops=8)
   { uint32_t v0=Data[0], v1=Data[1];                           // set up
     const uint32_t delta=0x9e3779b9; uint32_t sum=delta*Loops; // a key schedule constant
     uint32_t k0=Key[0], k1=Key[1], k2=Key[2], k3=Key[3];       // cache key
     for (int i=0; i < Loops; i++)                              // basic cycle start */
     { v1 -= ((v0<<4) + k2) ^ (v0 + sum) ^ ((v0>>5) + k3);
       v0 -= ((v1<<4) + k0) ^ (v1 + sum) ^ ((v1>>5) + k1);
       sum -= delta; }                                          // end cycle
     Data[0]=v0; Data[1]=v1;
   }

} ;


class OgnPosition
{ public:
   int FixQuality;              // 0 = none, 1 = GPS, 2 = Differential GPS (can be WAAS)
   int FixMode;                 // 1 = none, 2 = 2-D, 3 = 3-D
   int Satellites;

   int Year, Month, Day;      // Date from GPS
   int Hour, Min, Sec;        // Time-of-day (UTC) from GPS
   int FracSec;               // [1/1000 sec] some GPS-es give second fraction with the time-of-day
   uint32_t UnixTime;         // [sec] UNIX time (calc. from the above)

   int32_t Altitude;            // [0.1 meter]
   int32_t GeoidSeparation;     // [0.1 meter]
   int32_t Latitude;            // [0.0001/60 deg] about 0.018m accuracy, to convert to FLARM units mult by. 5/3
   int32_t Longitude;           // [0.0001/60 deg]
   int32_t PDOP;                // [0.1]
   int32_t HDOP;                // [0.1]
   int32_t VDOP;                // [0.1]

   int32_t Speed;               // [0.01 knot]
   int32_t Heading;             // [0.01 deg]

   int32_t ClimbRate;           // [0.1 meter/sec)
   int32_t TurnRate;            // [0.1 deg/sec]

   int32_t Temperature;         // [0.1 degC]

   int32_t DayTimeGGA;          // [sec]
   int32_t DayTimeRMC;          // [sec]

  public:

   OgnPosition() { Clear(); }

   void Clear(void)
   { FixQuality=0; FixMode=0; PDOP=0; HDOP=0; VDOP=0; UnixTime=0;
     Altitude=0; Latitude=0; Longitude=0;
     Speed=0; Heading=0; ClimbRate=0; TurnRate=0; Temperature=0;
     DayTimeGGA=(-1); DayTimeRMC=(-1); }

   bool isComplete(void) const                       // have both RMC and GGA sentences been received and for same time ?
   { if((DayTimeGGA<0) || (DayTimeRMC<0) ) return 0;
     if(DayTimeGGA!=DayTimeRMC) return 0;
     if(UnixTime==0) return 0;
     return 1; }

   bool isValid(void)                                // is GPS lock there ?
   { if(FixQuality==0) return 0;
     if(FixMode<2) return 0;
     if(UnixTime==0) return 0;
     if(Satellites<=0) return 0;
     return 1; }

   void PrintDateTime(void) const { printf("%02d.%02d.%04d %02d:%02d:%06.3f", Day, Month, Year, Hour, Min, Sec+0.001*FracSec ); }
   void PrintTime(void)     const { printf("%02d:%02d:%06.3f", Hour, Min, Sec+0.001*FracSec ); }

   int PrintDateTime(char *Out) const { return sprintf(Out, "%02d.%02d.%04d %02d:%02d:%02d.%03d", Day, Month, Year, Hour, Min, Sec, FracSec ); }
   int PrintTime(char *Out)     const { return sprintf(Out, "%02d:%02d:%02d.%03d", Hour, Min, Sec, FracSec ); }

   void Print(void) const
   { printf("Time/Date = "); PrintDateTime(); printf(" = %10ld.%03dsec\n", UnixTime, FracSec);
     printf("FixQuality=%d: %d satellites HDOP=%3.1f\n", FixQuality, Satellites, 0.1*HDOP);
     printf("Lat/Lon/Alt = [%+10.6f,%+10.6f]deg %+3.1f(%+3.1f)m\n", 0.0001/60*Latitude, 0.0001/60*Longitude, 0.1*Altitude, 0.1*GeoidSeparation);
     printf("Speed/Heading = %4.2fkt %06.2fdeg\n", 0.01*Speed, 0.01*Heading);
   }

   int Print(char *Out) const
   { int Len=0;
     Len+=sprintf(Out+Len, "Time/Date = "); Len+=PrintDateTime(Out+Len); Len+=sprintf(Out+Len, " = %10ld.%03dsec\n", UnixTime, FracSec);
     Len+=sprintf(Out+Len, "FixQuality=%d: %d satellites HDOP=%3.1f\n", FixQuality, Satellites, 0.1*HDOP);
     Len+=sprintf(Out+Len, "Lat/Lon/Alt = [%+10.6f,%+10.6f]deg %+3.1f(%+3.1f)m\n", 0.0001/60*Latitude, 0.0001/60*Longitude, 0.1*Altitude, 0.1*GeoidSeparation);
     Len+=sprintf(Out+Len, "Speed/Heading = %4.2fkt %06.2fdeg\n", 0.01*Speed, 0.01*Heading);
     return Len; }

   void PrintLine(void) const
   { PrintTime();
     printf(" %d/%d/%02d/%4.1f/%4.1f/%4.1f", FixQuality, FixMode, Satellites, 0.1*PDOP, 0.1*HDOP, 0.1*VDOP);
     printf(" [%+10.6f,%+10.6f]deg %+3.1f(%+3.1f)m", 0.0001/60*Latitude, 0.0001/60*Longitude, 0.1*Altitude, 0.1*GeoidSeparation);
     printf(" %4.1fkt %05.1fdeg", 0.01*Speed, 0.01*Heading);
     printf("\n"); }

   int PrintLine(char *Out) const
   { int Len=PrintDateTime(Out);
     Len+=sprintf(Out+Len, " %d/%d/%02d", FixQuality, FixMode, Satellites);
     Out[Len++]='/'; Len+=Format_UnsDec(Out+Len, PDOP, 2, 1);
     Out[Len++]='/'; Len+=Format_UnsDec(Out+Len, HDOP, 2, 1);
     Out[Len++]='/'; Len+=Format_UnsDec(Out+Len, VDOP, 2, 1);
     Out[Len++]=' ';
     Out[Len++]='['; Len+=Format_SignDec(Out+Len, Latitude/60, 6, 4);
     Out[Len++]=','; Len+=Format_SignDec(Out+Len, Longitude/60, 7, 4);
     Out[Len++]=']'; Out[Len++]='d'; Out[Len++]='e'; Out[Len++]='g';
     Out[Len++]=' '; Len+=Format_SignDec(Out+Len, Altitude, 4, 1); Out[Len++]='m';
     Out[Len++]='/'; Len+=Format_SignDec(Out+Len, GeoidSeparation, 4, 1); Out[Len++]='m';
     Out[Len++]=' '; Len+=Format_UnsDec(Out+Len, Speed/10  , 2, 1); Out[Len++]='k'; Out[Len++]='t';
     Out[Len++]=' '; Len+=Format_UnsDec(Out+Len, Heading/10, 4, 1); Out[Len++]='d'; Out[Len++]='e'; Out[Len++]='g';
     Out[Len++]='\n'; Out[Len++]=0; return Len; }

   int ReadNMEA(NMEA_RxMsg &RxMsg)
   {      if(RxMsg.isGPGGA()) return ReadGGA(RxMsg);
     else if(RxMsg.isGPRMC()) return ReadRMC(RxMsg);
     else if(RxMsg.isGPGSA()) return ReadGSA(RxMsg);
     else return 0; }

   int ReadNMEA(const char *NMEA)
   { int Err=0;
     Err=ReadGGA(NMEA); if(Err!=(-1)) return Err;
     Err=ReadGSA(NMEA); if(Err!=(-1)) return Err;
     Err=ReadRMC(NMEA); if(Err!=(-1)) return Err;
     return 0; }

   int ReadGGA(NMEA_RxMsg &RxMsg)
   { if(RxMsg.Parms!=14) return -1;
     DayTimeGGA = ReadTime((const char *)RxMsg.ParmPtr(0));
     FixQuality =ReadDec1(*RxMsg.ParmPtr(5)); if(FixQuality<0) FixQuality=0;
     Satellites=ReadDec2((const char *)RxMsg.ParmPtr(6)); if(Satellites<0) Satellites=0;
     // printf("FixQuality=%d: %d satellites\n", FixQuality, Satellites);
     ReadHDOP((const char *)RxMsg.ParmPtr(7));
     ReadLatitude(*RxMsg.ParmPtr(2), (const char *)RxMsg.ParmPtr(1));
     ReadLongitude(*RxMsg.ParmPtr(4), (const char *)RxMsg.ParmPtr(3));
     ReadAltitude(*RxMsg.ParmPtr(9), (const char *)RxMsg.ParmPtr(8));
     ReadGeoidSepar(*RxMsg.ParmPtr(11), (const char *)RxMsg.ParmPtr(10));
     return 1; }

   int ReadGGA(const char *GGA)
   { if(memcmp(GGA, "$GPGGA", 6)!=0) return -1;                   // check if the right sequence
     uint8_t Index[20]; if(IndexNMEA(Index, GGA)!=15) return -2;  // index parameters and check the sum

     DayTimeGGA = ReadTime(GGA+Index[1]);

     FixQuality =ReadDec1(GGA[Index[6]]); if(FixQuality<0) FixQuality=0;
     Satellites=ReadDec2(GGA+Index[7]); if(Satellites<0) Satellites=0;
     // printf("FixQuality=%d: %d satellites\n", FixQuality, Satellites);
     ReadHDOP(GGA+Index[8]);

     ReadLatitude( GGA[Index[3]], GGA+Index[2]);
     ReadLongitude(GGA[Index[5]], GGA+Index[4]);
     ReadAltitude(GGA[Index[10]], GGA+Index[9]);
     ReadGeoidSepar(GGA[Index[12]], GGA+Index[11]);

     // printf("ReadGGA() OK\n");
     return 0; }

   int ReadGSA(NMEA_RxMsg &RxMsg)
   { if(RxMsg.Parms!=17) return -1;
     FixMode =ReadDec1(*RxMsg.ParmPtr(1)); if(FixMode<0) FixMode=0;
     ReadPDOP((const char *)RxMsg.ParmPtr(14));
     ReadHDOP((const char *)RxMsg.ParmPtr(15));
     ReadVDOP((const char *)RxMsg.ParmPtr(16));
     return 1; }

   int ReadGSA(const char *GSA)
   { if(memcmp(GSA, "$GPGSA", 6)!=0) return -1;                   // check if the right sequence
     uint8_t Index[20]; if(IndexNMEA(Index, GSA)!=18) return -2;  // index parameters and check the sum
     FixMode =ReadDec1(GSA[Index[2]]); if(FixMode<0) FixMode=0;
     ReadPDOP(GSA+Index[15]);
     ReadHDOP(GSA+Index[16]);
     ReadVDOP(GSA+Index[17]);
     // printf("ReadGSA() OK\n");
     return 1; }

   int ReadRMC(NMEA_RxMsg &RxMsg)
   { if(RxMsg.Parms!=12) return -1;
     DayTimeRMC = ReadTime((const char *)RxMsg.ParmPtr(0));
     if(ReadDate((const char *)RxMsg.ParmPtr(8))>=0) UnixTime=CalcTime();
     ReadLatitude(*RxMsg.ParmPtr(3), (const char *)RxMsg.ParmPtr(2));
     ReadLongitude(*RxMsg.ParmPtr(5), (const char *)RxMsg.ParmPtr(4));
     ReadSpeed((const char *)RxMsg.ParmPtr(6));
     ReadHeading((const char *)RxMsg.ParmPtr(7));
     return 1; }

   int ReadRMC(const char *RMC)
   { if(memcmp(RMC, "$GPRMC", 6)!=0) return -1;                   // check if the right sequence
     uint8_t Index[20]; if(IndexNMEA(Index, RMC)!=13) return -2;  // index parameters and check the sum
     DayTimeRMC = ReadTime(RMC+Index[1]);
     if(ReadDate(RMC+Index[9])>=0) UnixTime=CalcTime();
     ReadLatitude( RMC[Index[4]], RMC+Index[3]);
     ReadLongitude(RMC[Index[6]], RMC+Index[5]);
     ReadSpeed(RMC+Index[7]);
     ReadHeading(RMC+Index[8]);
     // printf("ReadRMC() OK\n");
     return 1; }

   int calcDifferences(OgnPosition &RefPos) // calculate climb rate and turn ratewith an earlier reference position
   { ClimbRate=0; TurnRate=0;
     if(RefPos.FixQuality==0) return 0;
     // int TimeDiff=Sec-RefPos.Sec; if(TimeDiff<=0) TimeDiff+=60;
     int TimeDiff=UnixTime-RefPos.UnixTime; if(TimeDiff==0) return 0;
     ClimbRate=(Altitude-RefPos.Altitude)/TimeDiff;
     TurnRate=Heading-RefPos.Heading;
     if(TurnRate>18000) TurnRate-=36000; else if(TurnRate<(-18000)) TurnRate+=36000;
     TurnRate=(TurnRate)/(10*TimeDiff);
     return TimeDiff; }

   int Encode(OGN_Packet &Packet) const
   { Packet.setFixQuality(FixQuality<3 ? FixQuality:3);
     if((FixQuality>0)&&(FixMode>1)) Packet.setFixMode(FixMode-2);
                                else Packet.setFixMode(0);
     Packet.EncodeDOP(PDOP-10);
     int ShortTime=Sec;
     if(FracSec>500)
     { ShortTime+=1; if(ShortTime>=60) ShortTime-=60; }
     Packet.setTime(ShortTime);
     Packet.EncodeLatitude(Latitude);
     Packet.EncodeLongitude(Longitude);
     Packet.EncodeAltitude((Altitude+5)/10);
     Packet.EncodeSpeed((Speed+10)/20);
     Packet.EncodeHeading((Heading+5)/10);
     Packet.EncodeClimbRate(ClimbRate);
     Packet.EncodeTurnRate(TurnRate);
     // Packet.setFEC();  // calculate the error check
     return 0; }

  private:

   int ReadLatitude(char Sign, const char *Value)
   { int Deg=ReadDec2(Value); if(Deg<0) return -1;
     int32_t Mult; if(ReadFloat(Latitude, Mult, Value+2)<1) return -1;
     Latitude+=Deg*60*Mult;
     if(Sign=='S') Latitude=(-Latitude);
     else if(Sign!='N') return -1;
     // printf("Latitude = %+d/%d arc minute\n", Latitude, Mult);
     Latitude=Scale(Latitude, Mult, 10000); return 0; } // Latitude units: 0.0001/60 deg

   int ReadLongitude(char Sign, const char *Value)
   { int Deg=ReadDec3(Value); if(Deg<0) return -1;
     int32_t Mult; if(ReadFloat(Longitude, Mult, Value+3)<1) return -1;
     Longitude+=Deg*60*Mult;
     if(Sign=='W') Longitude=(-Longitude);
     else if(Sign!='E') return -1;
     // printf("Longitude = %+d/%d arc minute\n", Longitude, Mult);
     Longitude=Scale(Longitude, Mult, 10000); return 0; } // Longitude units: 0.0001/60 deg

   int ReadAltitude(char Unit, const char *Value)
   { if(Unit!='M') return -1;
     int32_t Mult; if(ReadFloat(Altitude, Mult, Value)<1) return -1;
     // printf("Altitude = %+d/%d meters\n", Altitude, Mult);
     Altitude=Scale(Altitude, Mult, 10); return 0; } // Altitude units: 0.1 meter

   int ReadGeoidSepar(char Unit, const char *Value)
   { if(Unit!='M') return -1;
     int32_t Mult; if(ReadFloat(GeoidSeparation, Mult, Value)<1) return -1;
     // printf("GeoidSepar = %+d/%d meters\n", GeoidSeparation, Mult);
     GeoidSeparation=Scale(GeoidSeparation, Mult, 10); return 0; } // GeoidSepar units: 0.1 meter

   int ReadPDOP(const char *Value)
   { int32_t Mult; if(ReadFloat(PDOP, Mult, Value)<1) return -1;
     // printf("PDOP = %+d/%d\n", PDOP, Mult);
     PDOP=Scale(PDOP, Mult, 10); return 0; }

   int ReadHDOP(const char *Value)
   { int32_t Mult; if(ReadFloat(HDOP, Mult, Value)<1) return -1;
     // printf("HDOP = %+d/%d\n", HDOP, Mult);
     HDOP=Scale(HDOP, Mult, 10); return 0; }

   int ReadVDOP(const char *Value)
   { int32_t Mult; if(ReadFloat(VDOP, Mult, Value)<1) return -1;
     // printf("VDOP = %+d/%d\n", VDOP, Mult);
     VDOP=Scale(VDOP, Mult, 10); return 0; }

   int ReadSpeed(const char *Value)
   { int32_t Mult; if(ReadFloat(Speed, Mult, Value)<1) return -1;
     // printf("Speed = %d/%d knots\n", Speed, Mult);
     Speed=Scale(Speed, Mult, 100); return 0; } // Speed units: 0.01 knots

   int ReadHeading(const char *Value)
   { int32_t Mult; if(ReadFloat(Heading, Mult, Value)<1) return -1;
     // printf("Heading = %d/%d knots\n", Heading, Mult);
     Heading=Scale(Heading, Mult, 100); return 0; } // Heading units: 0.01 degree

   int32_t ReadTime(const char *Value)
   { Hour=ReadDec2(Value);  if(Hour<0) return -1;
     Min=ReadDec2(Value+2); if(Min<0)  return -1;
     Sec=ReadDec2(Value+4); if(Sec<0)  return -1;
     if(Value[6]=='.')
     { int32_t Int, Mult;
       if(ReadFloat(Int, Mult, Value+6)>0)
       { // printf("FracSec=%d/%d\n", Int, Mult);
         FracSec=Scale(Int, Mult, 1000); }
     }
     return Hour*3600+Min*60+Sec; }

   int ReadDate(const char *Param)
   { Day=ReadDec2(Param);     if(Day<0)   return -1;
     Month=ReadDec2(Param+2); if(Month<0) return -1;
     Year=ReadDec2(Param+4);  if(Year<0)  return -1;
     if(Year>=70) Year+=1900; else Year+=2000;
     return 0; }

   time_t CalcTime(void) const
   { struct tm TM;
     // printf("CalcTime(): Date=%02d.%02d.%04d, Time=%02d.%02d.%06.3f", Day, Month, Year, Hour, Min, Sec+0.001*FracSec);
     TM.tm_hour = Hour; TM.tm_min = Min; TM.tm_sec = Sec;
     TM.tm_mday = Day; TM.tm_mon = Month-1; TM.tm_year = Year-1900;
     time_t Time = internal_timegm(&TM);
     // printf(" => %10d\n", Time);
     return Time; }

   int32_t static Scale(int32_t Value, int32_t Mult, int32_t DestMult)
   { if(DestMult==Mult) return Value;
     if(DestMult>Mult) return Value*(DestMult/Mult);
     return RoundDiv(Value, Mult/DestMult); }

   int32_t static RoundDiv(int32_t Value, int32_t Div)
   { return Value>0 ? (Value+Div/2)/Div : (Value-Div/2)/Div; }

   int static IndexNMEA(uint8_t Index[20], const char *Seq) // index parameters and verify the NMEA checksum
   { if(Seq[0]!='$') return -1;
     if(Seq[6]!=',') return -1;
     uint8_t Check=Seq[1]^Seq[2]^Seq[3]^Seq[4]^Seq[5]^Seq[6];
     Index[0]=1; Index[1]=7; int Params=2; int Ptr;
     for(Ptr=7; ; )
     { char ch=Seq[Ptr++]; if(ch<' ') return -1;
       if(ch=='*') break;
       Check^=ch;
       if(ch==',') { Index[Params++]=Ptr; }
     }
     if(Seq[Ptr++]!=HexDigit(Check>>4)) return -2;
     if(Seq[Ptr++]!=HexDigit(Check)) return -2;
     // printf("%s => [%d]\n", Seq, Params);
     return Params; }

  private:

   char static HexDigit(uint8_t Val)
   { Val&=0x0F; return Val<10 ? '0'+Val : 'A'+Val-10; }

   int static ReadDec1(char Digit)                // convert single digit into an integer
   { if(Digit<'0') return -1;                     // return -1 if not a decimal digit
     if(Digit>'9') return -1;
     return Digit-'0'; }

   int static ReadDec2(const char *Inp)           // convert two digit decimal number into an integer
   { int High=ReadDec1(Inp[0]); if(High<0) return -1;
     int Low=ReadDec1(Inp[1]); if(Low<0) return -1;
     return Low+10*High; }

   int static ReadDec3(const char *Inp)           // convert three digit decimal number into an integer
   { int High=ReadDec1(Inp[0]); if(High<0) return -1;
     int Mid=ReadDec1(Inp[1]);  if(Mid<0) return -1;
     int Low=ReadDec1(Inp[2]);  if(Low<0) return -1;
     return Low+10*Mid+100*High; }

   int static ReadUnsDec(int32_t &Int, const char *Inp)  // convert variable number of digits unsigned decimal number into an integer
   { Int=0; int Len=0;
     for( ; ; )
     { int Dig=ReadDec1(Inp[Len]); if(Dig<0) break;
       Int = 10*Int + Dig; Len++; }
     return Len; }                                // return number of characters read

   int static ReadSignDec(int32_t &Int, const char *Inp)
   { char Sign=Inp[0]; int Len=0;
     if((Sign=='+')||(Sign=='-')) Len++;
     Len+=ReadUnsDec(Int, Inp); if(Sign=='-') Int=(-Int);
     return Len; }

   int static ReadFloat(int32_t &Float, int32_t &Mult, const char *Inp)
   { char Sign=Inp[0]; int Len=0;
     if((Sign=='+')||(Sign=='-')) Len++;
     Len+=ReadUnsDec(Float, Inp); Mult=1;
     if(Inp[Len]!='.') goto Ret;
     Len++;
     for( ; ; )
     { int Dig=ReadDec1(Inp[Len]); if(Dig<0) break;
       Float = 10*Float + Dig; Len++; Mult*=10; }
     Ret: if(Sign=='-') Float=(-Float); return Len; }

  uint8_t static Format_UnsDec(char *Str, uint32_t Value, int MinDigits=1, int DecPoint=0)
  { uint32_t Base; uint8_t Pos, Len=0;
    for( Pos=10, Base=1000000000; Base; Base/=10, Pos--)
    { uint8_t Dig;
      if(Value>=Base)
      { Dig=Value/Base; Value-=Dig*Base; }
      else
      { Dig=0; }
      if(Pos==DecPoint) { (*Str++)='.'; Len++; }
      if( (Pos<=MinDigits) || (Dig>0) || (Pos<=DecPoint) )
      { (*Str++)='0'+Dig; Len++; MinDigits=Pos; }
    }
    return Len; }

  uint8_t static Format_SignDec(char *Str, int32_t Value, int MinDigits=1, int DecPoint=0)
  { if(Value<0) { (*Str++)='-'; Value=(-Value); }
           else { (*Str++)='+'; }
    return 1+Format_UnsDec(Str, Value, MinDigits, DecPoint); }

  private:

// copied from: http://stackoverflow.com/questions/16647819/timegm-cross-platform

   inline static int32_t is_leap(int32_t year)
   { if(year % 400 == 0) return 1;
     if(year % 100 == 0) return 0;
     if(year %   4 == 0) return 1;
     return 0; }

   inline static int32_t days_from_0(int32_t year)
   { year--; return 365 * year + (year / 400) - (year/100) + (year / 4); }

   inline static int32_t days_from_1970(int32_t year)
   { static const int days_from_0_to_1970 = days_from_0(1970);
     return days_from_0(year) - days_from_0_to_1970; }

   inline static int32_t days_from_1jan(int32_t year,int32_t month,int32_t day)
   { static const int32_t days[2][12] =
     {
       { 0,31,59,90,120,151,181,212,243,273,304,334},
       { 0,31,60,91,121,152,182,213,244,274,305,335}
     };
     return days[is_leap(year)][month-1] + day - 1; }

   inline static time_t internal_timegm(struct tm const *t)
   { int year = t->tm_year + 1900;
     int month = t->tm_mon;
     if(month > 11)
     { year += month/12;
       month %= 12; }
     else if(month < 0)
     { int years_diff = (-month + 11)/12;
       year -= years_diff;
       month+=12 * years_diff; }
     month++;
     int day = t->tm_mday;
     int day_of_year = days_from_1jan(year,month,day);
     int days_since_epoch = days_from_1970(year) + day_of_year;

     time_t seconds_in_day = 3600 * 24;
     time_t result = seconds_in_day * days_since_epoch + 3600 * t->tm_hour + 60 * t->tm_min + t->tm_sec;

     return result; }

} ;

#endif // of __OGN_H__

