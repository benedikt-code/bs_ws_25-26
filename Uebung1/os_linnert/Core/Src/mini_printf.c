#include <stdarg.h>
#include <stdint.h>
#include "uart.h"

// Helper: print Hexadezimal mit fixer anzahl an bits
static void uart_write_hex(uintptr_t value, unsigned num_hex_digits) {
    for (int i = (int)num_hex_digits - 1; i >= 0; --i) {
        unsigned hex_digit = (unsigned)((value >> (i*4)) & 0xF);
        char hex_char = (hex_digit < 10) ? ('0' + hex_digit) : ('a' + (hex_digit - 10));
        uart2_putc(hex_char);
    }
}

__attribute__((format(printf, 1, 2)))
int uart_printf(const char *format, ...) {
	// va_* for iterating the variable arguments
    va_list ap;
    va_start(ap, format);

    // iterieren durch jeden character in format
    for (const char *current_char = format; *current_char; ++current_char) {

    	// default case: kein % charakter
        if (*current_char != '%') {
            if (*current_char == '\n') uart2_putc('\r');
            uart2_putc(*current_char);
            continue;
        }

        // wenn % charakter: specifier anschauen
        ++current_char;
        switch (*current_char) {

        // %
        case '%': uart2_putc('%'); break;
        // character
        case 'c': {
            unsigned char_value = va_arg(ap, unsigned);
            uart2_putc((unsigned char)char_value);
            break;
        }
        // string
        case 's': {
            const char *string_value = va_arg(ap, const char *);
            // defensiv: falls NULL string
            if (!string_value) string_value = "(null)";
            uart2_write(string_value);
            break;
        }
        // hexadecimal
        case 'x': {
            unsigned int_value = va_arg(ap, unsigned);
            uart_write_hex((uintptr_t)int_value, 8);
            break;
        }
        // pointer
        case 'p': {
            uintptr_t pointer_value = (uintptr_t)va_arg(ap, void *);
            uart2_write("0x");
            uart_write_hex(pointer_value, (unsigned)(sizeof(uintptr_t) * 2));
            break;
        }
        // default case: kein spezifizierter character
        default:
            uart2_putc('%');
            // for the rare typo of "%\n"
            if (*current_char == '\n') uart2_putc('\r');
            uart2_putc(*current_char);
            break;
        }
    }
    va_end(ap);
    return 0;
}
