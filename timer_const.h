#ifndef __TIMER_CONST_H
#define __TIMER_CONST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------- defines -------- */
/* Timer identifiers */
#define HPT_TIMER_ID        1
#define PWR_DOWN_TIMER_ID   2
#define GPS_VALID_TIMER     3
#define DISP_GPS_TIMER      4
#define SP1_TIMER_ID        5
#define DISP_RX_TIMER       6
#define CTRL_TASK_TIMER_ID  7

/* miliseconds to timer tick macro */
#define TIMER_MS(x) (x)

#ifdef __cplusplus
}
#endif

#endif /* __TIMER_CONST_H */
