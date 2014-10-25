#ifndef __CONSOLE_H
#define __CONSOLE_H

#include <stdint.h>
#include <FreeRTOS.h>
#include <queue.h>
#include "cir_buf.h"

#ifdef __cplusplus
extern "C" {
#endif

void Console_Config(void);
void vTaskConsole(void* pvParameters);
void Console_SetNMEABuf(cir_buf_str* handle);
void Console_SetGPSQue(xQueueHandle* handle);

void Console_Send(const char* str, char block);

#ifdef __cplusplus
}
#endif

#endif /* __CONSOLE_H */
