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

// Public init für USART2 mit angefragter baud Rate
void uart2_init(uint32_t baud) {
	// Map Pins PA2 und PD6 zu USART2 mit AF7
    gpio_af7_usart2_pa2_pd6();

    // Aktivieren von USART2 peripheral clock on APB1 (Bus)
    // --> damit USART2-> Register ansteuerbar sind
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;


    // Sicherstellen: UART ist aus bevor wir format/baud ändern
    // UE = UART Enable
	// -> UE=0 (Sender/Empfänger werden intern deaktiviert)
	USART2->CR1 &= ~USART_CR1_UE;

    // Warten bis HW das quittiert
	// ISR = Interrupt & Status Register  - enthält Status-Flags.
	//TEACK / REACK are acknowledge bits that tell you when transmitter/receiver are enabled
    while (USART2->ISR & (USART_ISR_TEACK | USART_ISR_REACK)) { /* warten */ }

    // Character transmission procedure wie auf Seite 1329 im Manual

    // 1) Wortlänge/Parity/Oversampling setzen (CR1)
    // -> 8 Datenbits, keine Parität, Oversampling x16
    //M1 und M0 sind  = 00b für 8 Datenbits (word length), PCE = Parity Control Enable
    //PS = Partity Selection (even/odd) --> zur Sicherheit auch auf 0
    //OVER8 = oversampling auf 0b --> x16 oversampling
    USART2->CR1 &= ~(USART_CR1_M1 | USART_CR1_M0 | USART_CR1_PCE | USART_CR1_PS | USART_CR1_OVER8);


	// 2) Baudrate einstellen (BRR)
	// -> Formel für Oversampling x16: BRR = PCLK/baud
	uint32_t brr = (PCLK1_HZ + baud/2u) / baud;
	USART2->BRR = brr;

	// 3) Stopbits (CR2)
	// -> 00b = 1 Stopbit
	USART2->CR2 &= ~USART_CR2_STOP;

	// USART einschalten (UE)
	USART2->CR1 |= USART_CR1_UE;

	//5) DMA (CR3) – in dieser Übung aus
	USART2->CR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR | USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_ONEBIT | USART_CR3_EIE | USART_CR3_HDSEL);

	//6) Sender/Empfänger aktivieren
	// -> TE NACH UE setzen => erstes Frame ist IDLE (Manual)
	USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE);

	// Acknowledge-Bits abwarten (HW wirklich bereit)
	while ((USART2->ISR & USART_ISR_TEACK) == 0) { /* warten */ }
	while ((USART2->ISR & USART_ISR_REACK) == 0) { /* warten */ }
}

// Sendet 1 Zeichen (char c)
void uart2_putc(char c) {
	// Wenn Transmit Data Register (TDR) leer (=1): dann dürfen wir ein neues Byte schreiben
	// USART_ISR_TXE wechselt von 0 zu 1 und umgekehrt je nach Situation
    while (!(USART2->ISR & USART_ISR_TXE)) { /* wait */ }

    // Zeichen wird in das Senderegister geschrieben
    // Da wir word length 8 bits haben, casten wir das char auf int mit 8 bits
    USART2->TDR = (uint8_t)c;
}

//sendet einen C-String und nutzt dabei uart2_putc pro Zeichen
void uart2_write(const char *s) {
    while (*s) {
        if (*s == '\n') uart2_putc('\r');
        uart2_putc(*s++);
    }
}

int uart2_getc_blocking(void) {
	// Receive Data Register (RDR) nicht leer (=1): im RDR liegt ein empfangenes Byte
    while (!(USART2->ISR & USART_ISR_RXNE)) { /* wait */ }

    //Lesen Byte auf dem RDR Register und geben den int des Chars davon zurück (achten mit & 0xFF, dass wir nur 8 bits lesen)

    return (int)(USART2->RDR & 0xFF);
}
