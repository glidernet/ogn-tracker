/**
  ************************************
  * @file    cir_buf.c
  * @author  
  * @brief   Simple circular buffer
  ************************************
  */
	
#include <stdint.h>
#include <stddef.h>
#include "cir_buf.h"

static cir_buf_str cir_buf[CIR_BUF_NUM];
static uint8_t cir_buf_data[CIR_BUF_NUM][CIR_BUF_LEN];

/**
  * @brief  Init selected cir buffer, 
  * @param  buffer number: <0-CIR_BUF_NUM)
  * @retval addres of CIR buffer descriptor structure
  */
cir_buf_str* init_cir_buf(cir_buf_num buf_nr)
{
   cir_buf[buf_nr].buf_data = cir_buf_data[buf_nr];
   cir_buf[buf_nr].buf_len  = CIR_BUF_LEN;
   cir_buf[buf_nr].buf_ptr  = 0;
   
   return &cir_buf[buf_nr];
}

/**
  * @brief   Allocate space in selected cir buffer
  * @param   buf_str_ptr:pointer to buffer, data: data address, len: data length 
  * @retval  addres of data stored in cir buffer
  * @details Allocate space in selected cir buffer and put data to selected cir buffer.
  *          If source data pointer is non NULL then allocated buffer is populated with
  *          data pointed by uint8_t* data.
  */

uint8_t* cir_put_data(cir_buf_str* buf_str_ptr, uint8_t* data, uint16_t len)
{
   int i;
   uint16_t data_start_ptr;
   
   /* Check if there's enough data left in buffer to allocate 'len' bytes */
   if (len > buf_str_ptr->buf_len - buf_str_ptr->buf_ptr)
   {
      /* there is not enough data at the end of buffer -> reset pointer*/
      buf_str_ptr->buf_ptr = 0;
   }
   data_start_ptr = buf_str_ptr->buf_ptr;
   
   /* No source buffer defined - just reserve space of length len */
   if( NULL != data )
   {
      for (i=0; i<len; i++)
      {
         /* copy the data into buffer */
         buf_str_ptr->buf_data[data_start_ptr+i] = data[i];
      }
   }
   /* Update pointer data for the next call */
   buf_str_ptr->buf_ptr+= len;
   /* Reset pointer if ptr overrun */
   if (buf_str_ptr->buf_ptr >= buf_str_ptr->buf_len) buf_str_ptr->buf_ptr-= buf_str_ptr->buf_len;
   
   /* Return pointer to allocated buffer */
   return &buf_str_ptr->buf_data[data_start_ptr];
}
