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

/* this will be moved - it is not connected with SP1 hardware */
#define OGN_PKT_LEN       26           // OGN packet has 26 bytes of data
#define OGN_PKT_SYNC      0x0AF3656C   // OGN packet starts with these four bytes

/* --- Spirit task opcodes --- */
typedef enum
{
   SP1_SEND_OGN_PKT   = 1,  // Send packet in OGN format
}sp1_opcode_types;

/* --- SPIRIT1 related functions --- */
void Spirit1_Config(void);
void vTaskSP1(void* pvParameters);
xQueueHandle* Get_SP1Que();

#ifdef __cplusplus
}
#endif

#endif /* __SPIRIT1_H */
