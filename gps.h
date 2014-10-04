#ifndef __GPS_H
#define __GPS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t GPS_GetUnixTime(void);

void GPS_Config(void);
void vTaskGPS(void* pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* __GPS_H */
