#ifndef __OPTIONS_H
#define __OPTIONS_H

#include <stdint.h>

/* -------- enums -------- */
typedef enum
{
   OPT_CONS_SPEED = 1,
   OPT_GPS_SPEED,
   OPT_ACFT_ID,
   OPT_TX_POWER,
   OPT_XTAL_CORR,
   OPT_FREQ_OFS,
   OPT_IWDG,
   OPT_OPER_MODE,
   OPT_CHANNEL,
   OPT_GPSDUMP,
   OPT_MAX_TX_PWR,
   OPT_GPS_ALW_ON
} option_types;

typedef enum
{
   MODE_OGN = 0,
   MODE_CW
} oper_modes;

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
