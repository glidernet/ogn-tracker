#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>
#include <FreeRTOS.h>
#include <queue.h>

#ifdef __cplusplus
extern "C" {
#endif

void Control_Config(void);
void vTaskControl(void* pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_H */
