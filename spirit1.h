#ifndef __SPIRIT1_H
#define __SPIRIT1_H

#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- SPIRIT1 related functions --- */
void Spirit1_Config(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPIRIT1_H */
