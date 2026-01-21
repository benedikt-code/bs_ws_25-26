#include "exception_handlers.h"
#include "stm32l4xx.h"
#include "system_stm32l4xx.h"
#include "uart.h"
#include "os.h"
#include "syscalls.h"

void systick_init(void) {
    // Reload-Wert berechnen: Timer zählt (reload+1) Takte bis zum Interrupt
    uint32_t ticks = (SystemCoreClock / 1000u) * OS_TIME_SLICE_MS;
    if (ticks == 0) ticks = 1;
    // 1. Timer aufsetzen --> 2. Dann Timer auslösen
    // Startwert für Countdown (24-Bit, max 0xFFFFFF), wird automatisch bei 0 neu geladen
    SysTick->LOAD = ticks - 1u;
    // Val speichert aktuellen Zählerstand (Reihenfolge: Reload, reload-1, ..., 1, 0)
    // Zähler zurücksetzen und (Countflag in CTRL wird gelöscht)
    SysTick->VAL  = 0u;

    // Nested Vectored Interrupt Controller (NVIC) 
    // Niedrigere Priorität (14), höher als PendSV_IRQn (15)
    NVIC_SetPriority(SysTick_IRQn, 14);
    // Timer aktivieren: Bits 2-0 werden gesetzt: CLKSOURCE: | Tickint: Interrupt-Request an NVIC bei 0 | Enable (Timer aktiv)
    // Control and Status Register
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
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

    // Stackframe wählen: PSP (Thread) oder MSP (Handler)
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
    // SVC dispatcher: Syscall-Nummer wird nach unserer Konvention in gestacktem r12 gespeichert
    uint32_t *stack_frame = (uint32_t*)__get_PSP();
    uint32_t svc_number = stack_frame[4]; // gestacktes r12

    switch (svc_number) {
        case SYS_PUTCHAR: {
            int c = (int)stack_frame[0];
            uart2_putc((char)c);
            stack_frame[0] = 0; // Erfolg
        } break;

        case SYS_GETCHAR: {
            int r = os_k_getchar_blocking();
            if (r >= 0) {
                stack_frame[0] = (uint32_t)r;
            }
            // if r == -1 the thread got blocked; actual return will be injected by IRQ when data arrives
        } break;

        case SYS_THREAD_CREATE: {
            os_thread_fn_t fn = (os_thread_fn_t)stack_frame[0];
            void *arg = (void*)stack_frame[1];
            // Nutze _from_isr Version da wir bereits im Handler-Mode sind
            int tid = os_thread_create_from_isr(fn, arg);
            stack_frame[0] = (uint32_t)tid;
        } break;

        case SYS_THREAD_EXIT: {
            os_k_thread_exit(); // setzt state auf ZOMBIE und triggert PendSV
        } break;

        case SYS_SLEEP_MS: {
            uint32_t ms = stack_frame[0];
            os_k_sleep_blocking(ms);
            // blocked; return value will be irrelevant
        } break;

        default:
            uart_printf("Unknown SVC: %u\n", svc_number);
            break;
    }
}
// SysTick ISR
// Wird bei jedem Timer-underflow aufgerufen. ISR kurz halten --> nur Flags setzen
void SysTick_Handler(void) {
    // Update OS Zeit und fordere Kontextwechsel an
    os_tick_isr();
}