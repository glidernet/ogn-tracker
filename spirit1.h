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

/* --- Spirit task opcodes --- */
typedef enum
{
   SP1_SEND_OGN_PKT   = 1,  // Send packet in OGN format
   SP1_CHG_CHANNEL,         // Change active channel
}sp1_opcode_types;

/* --- SPIRIT1 related functions --- */
void Spirit1_Config(void);
void vTaskSP1(void* pvParameters);
xQueueHandle* Get_SP1Que();

#ifdef __cplusplus
}
#endif

#endif /* __SPIRIT1_H */
