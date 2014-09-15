#ifndef __GPS_H
#define __GPS_H

#include <stdint.h>

void GPS_Config(void);
void vTaskGPS(void* pvParameters);

#endif /* __GPS_H */
