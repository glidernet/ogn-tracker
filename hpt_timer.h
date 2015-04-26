#ifndef __HPT_TIMER_H
#define __HPT_TIMER_H

#include <stdint.h>
#include <FreeRTOS.h>
#include <queue.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------- defines -------- */

typedef enum
{
   HPT_RESTART = 0,  /* Restart the table */
   HPT_GPIO_UP,      /* Test GPIO up */
   HPT_GPIO_DOWN,    /* Test GPIO down */
   HPT_PREPARE_PKT,  /* Prepare OGN packet */
   HPT_COPY_PKT,     /* Copy OGN packet data */
   HPT_SP1_CHANNEL,  /* Switch SP1 to selected channel */
   HPT_TX_PKT,       /* TX copied packet data */
   HPT_TX_PKT_LBT,   /* TX copied packet data with Listen Before Talk and random access */      
   HPT_IWDG_RELOAD   /* Reload Independent Watchdog */   
} hpt_opcodes;

/* -------- structures ------- */
typedef struct
{
   uint32_t     time;
   hpt_opcodes  opcode;
   uint32_t     data1;
} HPT_Event;

/* -------- functions -------- */
void HPT_Config(void);
void HPT_Start(HPT_Event* hpt_table);
BaseType_t HPT_RestartFromISR(void);
void HPT_Debug(uint8_t state);

#ifdef __cplusplus
}
#endif

#endif /* __HPT_TIMER_H */
