#pragma once
#include <stdint.h>

void uart2_init(uint32_t baud);
void uart2_putc(char c);
void uart2_write(const char *s);
int  uart2_getc_blocking(void);

int mini_printf(const char *fmt, ...);
