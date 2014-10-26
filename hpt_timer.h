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
   HPT_GPIO_DOWN,    /* Test GPIO down */
   HPT_PREPARE_PKT,  /* Prepare OGN packet */
   HPT_SEND_PKT,     /* Send OGN packet */
   HPT_SP1_CHANNEL,  /* Switch SP1 to selected channel */ 
   HPT_IWDG_RELOAD   /* Reload Independent Watchdog */   
} htp_opcodes;

/* -------- structures ------- */
typedef struct
{
   uint32_t     time;
   htp_opcodes  opcode;
   uint32_t     data1;
} HPT_Event;

/* -------- functions -------- */
void HPT_Config(void);
void HPT_Start(HPT_Event* hpt_table);
BaseType_t HPT_RestartFromISR(void);

#ifdef __cplusplus
}
#endif

#endif /* __HPT_TIMER_H */
