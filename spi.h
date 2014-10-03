#ifndef __SPI_H
#define __SPI_H

#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- SPI1 related functions --- */
void SPI1_Config(void);
void SPI1_Send(uint8_t* data_tx, uint8_t* data_rx, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H */
