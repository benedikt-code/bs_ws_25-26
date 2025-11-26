#pragma once
#include <stdint.h>

void uart2_init(uint32_t baud);
void uart2_putc(char c);
void uart2_write(const char *s);
int  uart2_getc_blocking(void);
int uart2_getc_nonblocking(void);

void uart2_enable_rx_irq(void);

int uart_printf(const char *fmt, ...);
void USART2_IRQHandler(void);
