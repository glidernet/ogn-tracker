#ifndef __MESSAGES_H
#define __MESSAGES_H

#include <stdint.h>

typedef enum
{
   CONSOLE_USART_SRC_ID = 1,
   GPS_USART_SRC_ID
} src_id_types;

typedef struct
{
   uint8_t     src_id;
   uint8_t     msg_opcode;
   uint16_t    msg_len;
   uint32_t    msg_data;
}task_message;

#endif /* __MESSAGES_H */
