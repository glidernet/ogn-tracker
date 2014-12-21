#ifndef __DISPLAY_H
#define __DISPLAY_H

#include <stdint.h>
#include <FreeRTOS.h>
#include <queue.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------- API defines -------- */
typedef enum
{
   DISP_GPS_NO_FIX  = 1,   // Lost GPS fix
   DISP_GPS_FIX,           // Acquired GPS fix
}display_opcode_types;

/* -------- API functions -------- */
void Display_Config(void);
void vTaskDisplay(void* pvParameters);
xQueueHandle* Get_DisplayQue();

#ifdef __cplusplus
}
#endif

#endif /* __DISPLAY_H */
