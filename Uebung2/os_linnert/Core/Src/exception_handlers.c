/*
 * exception_handlers.c
 *
 *  Created on: Nov 19, 2025
 *      Author: andre
 */

#include "exception_handlers.h"
#include "stm32l4xx.h"
#include "uart.h"

// Stack-Größen definieren
#define MSP_STACK_SIZE  0x1000  // 4KB für System/Handler
#define PSP_STACK_SIZE  0x800   // 2KB für Anwendung

// Stack-Bereiche im RAM definieren (statisch allokiert)
static uint8_t msp_stack[MSP_STACK_SIZE] __attribute__((aligned(8)));
static uint8_t psp_stack[PSP_STACK_SIZE] __attribute__((aligned(8)));

void init_stacks(void) {
    // MSP auf Ende des MSP-Stack-Bereichs setzen
    uint32_t msp_top = (uint32_t)&msp_stack[MSP_STACK_SIZE];
    __set_MSP(msp_top);

    // PSP auf Ende des PSP-Stack-Bereichs setzen
    uint32_t psp_top = (uint32_t)&psp_stack[PSP_STACK_SIZE];
    __set_PSP(psp_top);

    // CONTROL Register setzen: PSP für Thread Mode verwenden
    __set_CONTROL(__get_CONTROL() | 0x02);  // SPSEL = 1 (PSP aktiv)
    __ISB(); // Instruction Synchronization Barrier
}

void test_exceptions(char command) {
    switch(command) {
    	case '1':
    		uart_printf("Triggering UsageFault (division by zero)...\n");
			{
				volatile int a = 1;
				volatile int b = 0;
				volatile int c = a / b;
				// (void)c; Ich glaube das brauchen wir nicht
			}
        case '2':
            uart_printf("Triggering UsageFault (undefined instruction)...\n");
            __asm volatile(".word 0xDE00"); // Ungültige 32-bit Instruktion
            break;

        case '3': // BusFault - Imprecise data bus error
            uart_printf("Triggering BusFault (invalid address access)...\n");
            *(volatile uint32_t*)0xFFFFFFFF = 0x12345678;
            break;

        case '4': // SVC Call
            uart_printf("Triggering SVC call...\n");
            __asm volatile("svc #42");
            break;

        default:
            uart_printf("Unknown command. Use 1, 2, 3 or 4.\n");
            break;
    }
}


void NMI_Handler(void) {
    uart_printf("\n=== NON-MASKABLE INTERRUPT ===\n");
    uart_printf("Critical hardware event occurred\n");
    // WFI - WaitForInterrupt, ist nen Low Power Mode, wir könnten nen reset vom System machen über "Watchdog"?
    while(1) { __WFI(); }
}

void HardFault_Handler(void) {
    uart_printf("\n=== HARD FAULT ===\n");

    // Fault-Status lesen
    // SCB - System Control Block
    // HFSR - Hard Fault Status Register
    // (32 Bit groß,
    // 0-29 und 31 sind reserved,
    // 30 ist Forced also anderer Fault wurde nicht behandelt,
    // 1 ist VECTTBL Fehler beim Vector Table Zugriff)
    uint32_t hfsr = SCB->HFSR;
    uart_printf("HFSR: 0x%x\n", hfsr);

    // benutzen Masken um die HardFaults zu klassifizieren (Die sind ausm CMSIS)
    if (hfsr & SCB_HFSR_FORCED_Msk) {
        uart_printf("Forced HardFault (escalated from configurable fault)\n");
    }
    if (hfsr & SCB_HFSR_VECTTBL_Msk) {
        uart_printf("Vector table read fault\n");
    }

    // Stack Frame analysieren (Ich habe bei den SVC sachen immer den PSP genommen, weil ich immer errors bekommen habe,
    // aber bei dir hatten die Sachen ja funktioniert, deshalb lass ich das mal so noch. Sollten wir nochmal nachtesten !TEST?)
    uint32_t *stack_frame;
    if (__get_CONTROL() & 0x02) {
        stack_frame = (uint32_t*)__get_PSP();
    } else {
        stack_frame = (uint32_t*)__get_MSP();
    }

    uart_printf("PC: 0x%x\n", stack_frame[6]);
    uart_printf("LR: 0x%x\n", stack_frame[5]);
    uart_printf("SP: 0x%p\n", stack_frame);

    // WFI - WaitForInterrupt, ist nen Low Power Mode, wir könnten nen reset vom System machen über "Watchdog"?
    while(1) { __WFI(); }
}

void MemManage_Handler(void) {
    uart_printf("\n=== MEMORY MANAGEMENT FAULT ===\n");

    // SCB - System Control Block
    // CFSR - Configurable Fault Status Register (32 Bit groß)
    // 0-7 MMFSR - Memory Management Fault Status Register
    // 8-15 BFSR - Bus Fault Status Register
    // 16-31 UFSR - Usage Fault Status Register
    // jeder Bit steht hier für einen bestimmten Fehler

    // wir nehmen immer das volle CFSR Register für die Masks (speicher sparender wäre die Masks zu shiften)
    uint32_t mmfsr = SCB->CFSR & 0xFFFFFFFF;
    uart_printf("MMFSR: 0x%x\n", mmfsr);

    // Wir benutzen Masken um manche Memory Faults zu klassifizieren (Die sind ausm CMSIS)
    if (mmfsr & SCB_CFSR_MMARVALID_Msk) {
        uart_printf("Fault Address: 0x%x\n", SCB->MMFAR);
    }
    if (mmfsr & SCB_CFSR_DACCVIOL_Msk) {
        uart_printf("Data access violation\n");
    }
    if (mmfsr & SCB_CFSR_IACCVIOL_Msk) {
        uart_printf("Instruction access violation\n");
    }

    // WFI - WaitForInterrupt, ist nen Low Power Mode, wir könnten nen reset vom System machen über "Watchdog"?
    while(1) { __WFI(); }
}

void UsageFault_Handler(void) {
    uart_printf("\n=== USAGE FAULT ===\n");

    // SCB - System Control Block
    // CFSR - Configurable Fault Status Register (32 Bit groß)
    // 0-7 MMFSR - Memory Management Fault Status Register
    // 8-15 BFSR - Bus Fault Status Register
    // 16-31 UFSR - Usage Fault Status Register
    // jeder Bit steht hier für einen bestimmten Fehler

    // wir nehmen immer das volle CFSR Register für die Masks (speicher sparender wäre die Masks zu shiften)
    uint32_t ufsr = (SCB->CFSR) & 0xFFFFFFFF;

    uart_printf("UFSR: 0x%x\n", ufsr);

    // Wir benutzen Masken um Usage Faults zu klassifizieren (Die sind ausm CMSIS)
    if (ufsr & SCB_CFSR_UNDEFINSTR_Msk) {
        uart_printf("Undefined instruction\n");
    }
    if (ufsr & SCB_CFSR_INVSTATE_Msk) {
        uart_printf("Invalid state\n");
    }
    if (ufsr & SCB_CFSR_INVPC_Msk) {
        uart_printf("Invalid PC load\n");
    }
    if (ufsr & SCB_CFSR_NOCP_Msk) {
        uart_printf("No coprocessor\n");
    }
    if (ufsr & SCB_CFSR_DIVBYZERO_Msk) {
        uart_printf("Division by zero\n");
    }

    // WFI - WaitForInterrupt, ist nen Low Power Mode, wir könnten nen reset vom System machen über "Watchdog"?
    while(1) { __WFI(); }
}

void BusFault_Handler(void) {
    uart_printf("\n=== BUS FAULT ===\n");

    // SCB - System Control Block
    // CFSR - Configurable Fault Status Register (32 Bit groß)
    // 0-7 MMFSR - Memory Management Fault Status Register
    // 8-15 BFSR - Bus Fault Status Register
    // 16-31 UFSR - Usage Fault Status Register
    // jeder Bit steht hier für einen bestimmten Fehler

    // wir nehmen immer das volle CFSR Register für die Masks (speicher sparender wäre die Masks zu shiften)

    uint32_t bfsr = SCB->CFSR & 0xFFFFFFFF;
    uart_printf("BFSR: 0x%x\n", bfsr);

    // Wir benutzen Masken um Bus Faults zu klassifizieren (Die sind ausm CMSIS)
    if (bfsr & SCB_CFSR_BFARVALID_Msk) {
        uart_printf("Fault Address: 0x%x\n", SCB->BFAR);
    }
    if (bfsr & SCB_CFSR_PRECISERR_Msk) {
        uart_printf("Precise data bus error\n");
    }
    if (bfsr & SCB_CFSR_IMPRECISERR_Msk) {
        uart_printf("Imprecise data bus error\n");
    }
    if (bfsr & SCB_CFSR_IBUSERR_Msk) {
        uart_printf("Instruction bus error\n");
    }

    // WFI - WaitForInterrupt, ist nen Low Power Mode, wir könnten nen reset vom System machen über "Watchdog"?
    while(1) { __WFI(); }
}



void SVC_Handler(void) {
    uart_printf("\n=== SUPERVISOR CALL ===\n");

    // SVC-Nummer aus der Instruktion extrahieren
    // Hier ist das Gegenstück zum HardFault_Handler !TEST?
    uint32_t *stack_frame = (uint32_t*)__get_PSP();
    uint32_t pc = stack_frame[6];
    uint16_t *svc_instruction = (uint16_t*)(pc - 2);
    uint8_t svc_number = *svc_instruction & 0xFF;

    // Ich lass das hier mal noch auskommentiert, weil es so war aber ich glaube die waren auch nur bei mir direkt problematisch !TEST?
    //uart_printf("SVC Number: 0x%x\n", svc_number);
    //uart_printf("Called from PC: 0x%x\n", pc);

    // implementation für SVCs mit einem Beispiel
    switch(svc_number) {
        case 42:
            uart_printf("SVC 42 called\n");
            break;
        default:
            uart_printf("Unknown SVC\n");
            break;
    }

    // wir brauchen keine endlosschleife weil es nur sagt, wir machen hier nen supervisor call
}
