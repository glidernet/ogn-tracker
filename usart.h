#ifndef __USART_H
#define __USART_H

#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

void USART2_Config(void);
void USART2_SetQue(xQueueHandle* handle);
void USART2_Send(uint8_t* data, uint16_t len);
void USART2_Wait(void);

#endif /* __USART_H */
