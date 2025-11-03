#include <stdarg.h>
#include <stdint.h>
#include "uart.h"

static void put_hex_nibbles(uintptr_t v, unsigned n) {
    for (int i = (int)n - 1; i >= 0; --i) {
        unsigned nib = (unsigned)((v >> (i*4)) & 0xF);
        char ch = (nib < 10) ? ('0' + nib) : ('a' + (nib - 10));
        uart2_putc(ch);
    }
}

int mini_printf(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    for (const char *p = fmt; *p; ++p) {
        if (*p != '%') {
            if (*p == '\n') uart2_putc('\r');
            uart2_putc(*p);
            continue;
        }
        ++p;
        switch (*p) {
        case '%': uart2_putc('%'); break;
        case 'c': {
            unsigned v = va_arg(ap, unsigned);
            uart2_putc((unsigned char)v);
            break;
        }
        case 's': {
            const char *s = va_arg(ap, const char *);
            if (!s) s = "(null)";
            uart2_write(s);
            break;
        }
        case 'x': {
            unsigned v = va_arg(ap, unsigned);
            put_hex_nibbles((uintptr_t)v, 8);
            break;
        }
        case 'p': {
            uintptr_t v = (uintptr_t)va_arg(ap, void *);
            uart2_write("0x");
            put_hex_nibbles(v, (unsigned)(sizeof(uintptr_t) * 2));
            break;
        }
        default:
            uart2_putc('%'); uart2_putc(*p); break;
        }
    }
    va_end(ap);
    return 0;
}
