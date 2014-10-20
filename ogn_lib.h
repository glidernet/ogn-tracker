#ifndef __OGN_LIB_H
#define __OGN_LIB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------- OGN defines -------- */
#define OGN_PKT_LEN       26           // OGN packet has 26 bytes of data
#define OGN_PKT_SYNC      0x0AF3656C   // OGN packet starts with these four bytes

/* -------- OGN exported functions -------- */
uint8_t  OGN_Init(void);                                // initialize
void     OGN_SetAcftID(uint32_t id);                    // set Aircraft identificatin
uint32_t OGN_GetPosition(char *Output);                 // get GPS position in a string: to be displayed in the console
int      OGN_Parse_NMEA(const char* str, uint8_t len);  // process an NMEA sentence from the GPS
uint8_t* OGN_PreparePacket(void);                       // make an OGN packet

#ifdef __cplusplus
}
#endif

#endif /* __OGN_LIB_H */
