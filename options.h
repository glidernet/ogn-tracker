#ifndef __OPTIONS_H
#define __OPTIONS_H

#include <stdint.h>

/* -------- enums -------- */
typedef enum
{
   OPT_CONS_SPEED = 1,
   OPT_GPS_SPEED,
   OPT_ACFT_ID
} option_types;

#ifdef __cplusplus
extern "C" {
#endif

/* -------- function prototypes -------- */
void InitOptions(void);
void ResetOptions(void);
uint8_t WriteOptions(void);

void* GetOption(option_types opt_code);
void  SetOption(option_types opt_code, void* value);

#ifdef __cplusplus
}
#endif

#endif /* __OPTIONS_H */
