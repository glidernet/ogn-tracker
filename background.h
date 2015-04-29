#ifndef __BACKGROUND_H
#define __BACKGROUND_H

#include <stdint.h>
#include <FreeRTOS.h>
#include <queue.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------- API defines -------- */
typedef enum
{
   BKGRD_ADC_MEASURE = 0  /* Perform ADC conversions for voltages and CPU temperature */
} bkgrd_opcodes;

/* -------- API functions -------- */
void Background_Config(void);
void vTaskBackground(void* pvParameters);
xQueueHandle* Get_BackgroundQue();
int16_t BKGRD_Get_Volt_VDD();
int16_t BKGRD_Get_Volt_VBat();

#ifdef __cplusplus
}
#endif

#endif /* __BACKGROUND_H */
