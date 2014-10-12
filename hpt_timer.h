#ifndef __HPT_TIMER_H
#define __HPT_TIMER_H

#include <stdint.h>
#include <FreeRTOS.h>
#include <queue.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------- defines -------- */
#define HPT_MS(x) (x)

/* -------- functions -------- */
void HPT_Config(void);
void HPT_Start(void);

#ifdef __cplusplus
}
#endif

#endif /* __HPT_TIMER_H */
