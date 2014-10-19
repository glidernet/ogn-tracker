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
uint8_t  OGN_Init(void);
void     OGN_SetAcftID(uint32_t id);
uint32_t OGN_GetPosition(char *Output);
void     OGN_Parse_NMEA(const char* str, uint8_t len);
uint8_t* OGN_PreparePacket(void);

#ifdef __cplusplus
}
#endif

#endif /* __OGN_LIB_H */
