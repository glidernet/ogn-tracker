#ifndef __USART_H
#define __USART_H

#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include "cir_buf.h"

/* --- USART2 related functions --- */
void USART2_Config(uint32_t speed);
void USART2_SetQue(xQueueHandle* handle);
void USART2_Send(uint8_t* data, uint16_t len);
void USART2_Wait(void);

/* --- USART3 related functions --- */
void USART3_Config(uint32_t speed);
void USART3_SetQue(xQueueHandle* handle);
void USART3_SetBuf(cir_buf_str* handle);

#endif /* __USART_H */
