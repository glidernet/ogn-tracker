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

typedef enum
{
   HPT_END      = 1, /* End the table */
   HPT_RESTART,      /* Restart the table */
   HPT_GPIO_UP,      /* Test GPIO up */
   HPT_GPIO_DOWN     /* Test GPIO down */
} htp_opcodes;

/* -------- structures ------- */
typedef struct
{
   uint32_t    time;
   htp_opcodes opcode;
} HPT_Event;

/* -------- functions -------- */
void HPT_Config(void);
void HPT_Start(HPT_Event* hpt_table);
void HPT_Restart(void);

#ifdef __cplusplus
}
#endif

#endif /* __HPT_TIMER_H */
