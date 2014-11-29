#ifndef __SPIRIT1_H
#define __SPIRIT1_H

#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------- defines -------- */

/* Maximum allowable by SPIRIT1 Library TX power settings. */
/* Constants taken from SPIRIT_Radio.h: */
/* #define IS_PAPOWER_DBM(PATABLE)      ((PATABLE)>= (-31) && (PATABLE)<=(12)) */

#define SPIRIT1_LIB_MAX_POWER (+12.0F)
#define SPIRIT1_LIB_MIN_POWER (-31.0F)

/* --- Spirit task opcodes --- */
typedef enum
{
   SP1_SEND_OGN_PKT   = 1,  // Send packet in OGN format
   SP1_CHG_CHANNEL,         // Change active channel
   SP1_START_CW,            // Start transmitting continuous wave
   SP1_STOP_CW              // Stop transmitting continuous wave
}sp1_opcode_types;

/* --- SPIRIT1 related functions --- */
void Spirit1_Config(void);
void vTaskSP1(void* pvParameters);
xQueueHandle* Get_SP1Que();

#ifdef __cplusplus
}
#endif

#endif /* __SPIRIT1_H */
