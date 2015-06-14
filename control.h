#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>
#include <FreeRTOS.h>
#include <queue.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SHDN_MAGIC_NUM (1234)
#define SHDN_REG_NUM   (0)

void Control_Config(void);
void vTaskControl(void* pvParameters);
xQueueHandle* Get_ControlQueue();
void PreShutDownSequence(void);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_H */
