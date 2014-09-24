#ifndef __OPTIONS_H
#define __OPTIONS_H
#include <stdint.h>

/* -------- enums -------- */
typedef enum
{
   OPT_CONS_SPEED = 1,
   OPT_GPS_SPEED
}option_types;

/* -------- function prototypes -------- */
void InitOptions(void);
void ResetOptions(void);
uint8_t WriteOptions(void);

void* GetOption(option_types opt_code);

#endif /* __OPTIONS_H */
