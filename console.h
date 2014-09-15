#ifndef __CONSOLE_H
#define __CONSOLE_H

#include <stdint.h>
#include <FreeRTOS.h>
#include <queue.h>
#include "cir_buf.h"

void Console_Config(void);
void vTaskConsole(void* pvParameters);
void Console_SetNMEABuf(cir_buf_str* handle);
void Console_SetGPSQue(xQueueHandle* handle);

#endif /* __CONSOLE_H */
