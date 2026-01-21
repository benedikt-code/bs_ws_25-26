#pragma once
#include <stdint.h>

void uart2_init(uint32_t baud);
void uart2_putc(char c);
void uart2_write(const char *s);

int  uart2_getc_blocking(void);
int  uart2_getc_nonblocking(void);

void uart2_enable_rx_irq(void);

__attribute__((format(printf, 1, 2)))
int uart_printf(const char *fmt, ...);

// RX callback aufgerufen von USART2 IRQ nachdem Erhalten eines Bytes
// Rückgabe: 1 wenn Byte konsumiert (nicht in Ringbuffer), 0 sonst
typedef int (*uart2_rx_callback_t)(uint8_t b);
void uart2_set_rx_callback(uart2_rx_callback_t cb);

// IRQ handler 
void USART2_IRQHandler(void);
