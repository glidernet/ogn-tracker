/**
  ************************************
  * @file    cir_buf.h 
  * @author  
  * @brief   Simple circular buffer
  ************************************
  */
	
#ifndef __CIR_BUF_H
#define __CIR_BUF_H

#include <stdint.h>

#define CIR_BUF_LEN  512

typedef enum
{
   CIR_BUF_NMEA = 0,
   CIR_BUF_NUM
} cir_buf_num;

typedef struct {
   uint8_t*  buf_data;
   uint16_t  buf_ptr;
   uint16_t  buf_len;
} cir_buf_str;

#ifdef __cplusplus
extern "C" {
#endif

cir_buf_str* init_cir_buf(cir_buf_num buf_nr);
uint8_t*     cir_put_data(cir_buf_str* buf_str_ptr, uint8_t* data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __CIR_BUF_H */
