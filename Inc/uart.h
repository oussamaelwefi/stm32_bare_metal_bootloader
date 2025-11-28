#ifndef __UART_H__
#define __UART_H__

#include "stm32f4xx.h"

void uart_init(void);
uint8_t uart_read(void);
void bootloader_send_ack(uint8_t follow_len);
void bootloader_send_nack(void);
void bootloader_uart_write_data(uint8_t *pBuffer, uint32_t len);
void uart3_write(int ch);
void uart3_init(void);


#endif
