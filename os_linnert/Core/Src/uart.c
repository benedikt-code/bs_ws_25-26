#include "stm32l4xx.h"
#include <stdint.h>

#ifndef PCLK1_HZ
#define PCLK1_HZ 4000000u  // MSI ~4 MHz at reset
#endif

static inline void gpio_af7_usart2_pa2_pd6(void) {
	//Bus AHB2ENR Zugriff - Ports A und D "freischalten"(für Pin A2, D6)
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIODEN;

    // PA2 und PD6 werden zur Alternate Function 7 gemapped
    // diese ist widerum mit USART2 gemapped --> kein GPIO Pin mehr
    // PA2 and PD6 should stop acting like general GPIOs
    // and instead route signals to/from USART2

    // PA2 -> AF7 (USART2_TX)
    /* MODER sagt wie unser Pin sich verhalten soll (4 Modes)
     * 00 Input Mode
     * 01 General Purpose Output
     * 10 Alternate function
     * 11 Analog Mode */
    GPIOA->MODER   &= ~(0x3u << (2*2));
    GPIOA->MODER   |=  (0x2u << (2*2));
    /* AFR entscheidet, welche Alternate Function wir nutzen
     * [0] bezieht sich auf die Pins 0-7
     * [1] bezieht sich auf die Pins 8-15
     * Für jeden Pin sind 4-bits vorgesehen, die wir alle löschen
     * und dann mit der alternate Function belegen die wir brauchen (0-15)*/
    GPIOA->AFR[0]  &= ~(0xFu << (4*2));
    GPIOA->AFR[0]  |=  (0x7u << (4*2));
    /* OSPEEDR sagt wie schnell unser Pin den output state switchen kann (4 Modes)
	 * 00 slow (für Leds GPIOs)
	 * 01 medium (für General I/O)
	 * 10 fast
	 * 11 super fast (für UART */
    // Wir müssen nicht clearen weil wir eh mit 11 ver"odern"
    GPIOA->OSPEEDR |=  (0x3u << (2*2));
    /* OTYPER there are two types 0 push pull und 1 open drain für Uart nutzt man push pull */
    GPIOA->OTYPER  &= ~(1u   << 2);
    /* PUPDR sagt wie stellt die pull up pull down resistor (4 Modes)
	 * 00 pullup/down deactivated
	 * 01 nur pull up
	 * 10 nur pull down
	 * 11 reserved (hardware unterstützt das nicht, ist nicht setzbar)
	 * Wir deaktivieren es */
    GPIOA->PUPDR   &= ~(0x3u << (2*2));

    // PD6 -> AF7 (USART2_RX)
    GPIOD->MODER   &= ~(0x3u << (2*6));
    GPIOD->MODER   |=  (0x2u << (2*6));
    GPIOD->AFR[0]  &= ~(0xFu << (4*6));
    GPIOD->AFR[0]  |=  (0x7u << (4*6));
    GPIOD->OSPEEDR |=  (0x3u << (2*6));
    GPIOD->OTYPER  &= ~(1u   << 6);
    GPIOD->PUPDR   &= ~(0x3u << (2*6));
}

void uart2_init(uint32_t baud) {
    gpio_af7_usart2_pa2_pd6();

    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

    USART2->CR1 = 0; // disable before config
    uint32_t brr = (PCLK1_HZ + baud/2u) / baud;  // oversampling by 16
    USART2->BRR = brr;

    USART2->CR1 = USART_CR1_TE | USART_CR1_RE; // 8N1, no parity
    USART2->CR1 |= USART_CR1_UE;

    while ((USART2->ISR & (USART_ISR_TEACK | USART_ISR_REACK))
            != (USART_ISR_TEACK | USART_ISR_REACK)) { /* wait */ }
}

void uart2_putc(char c) {
    while (!(USART2->ISR & USART_ISR_TXE)) { /* wait */ }
    USART2->TDR = (uint8_t)c;
}

void uart2_write(const char *s) {
    while (*s) {
        if (*s == '\n') uart2_putc('\r');
        uart2_putc(*s++);
    }
}

int uart2_getc_blocking(void) {
    while (!(USART2->ISR & USART_ISR_RXNE)) { /* wait */ }
    return (int)(USART2->RDR & 0xFF);
}
