#include "exception_handlers.h"
#include "stm32l4xx.h"
#include "system_stm32l4xx.h"
#include "stm32l496xx.h"
#include "uart.h"

// Timer-Variablen (definiert in main.c)
// Systemzeit in Ticks
extern volatile uint32_t g_ticks;
// Flag für periodische Events
extern volatile uint8_t  g_timer_event;

// Frequenz der "!"-Ausgabe
// 8x pro Sekunde = alle 125ms
#define TIMER_EVENT_HZ 8u

// Stack-Größen definieren
#define MSP_STACK_SIZE  0x1000  // 4KB für System/Handler
#define PSP_STACK_SIZE  0x800   // 2KB für Anwendung

// Stack-Bereiche im RAM definieren (statisch allokiert)
// Startadresse muss durch 8 teilbar sein (ARM-Anforderung)
static uint8_t msp_stack[MSP_STACK_SIZE] __attribute__((aligned(8)));
static uint8_t psp_stack[PSP_STACK_SIZE] __attribute__((aligned(8)));

/*
Allgemeines Layout (fully descending)
RAM Layout (256 KB von 0x20000000 bis 0x20040000):
MSP ganz oben (Ende des RAMs), danach PSP
HOHE Adressen (Ende des RAMs)
↑
│  0x20040000 ← _estack (Stack Pointer zeigt HIERHIN beim Start)
│              ← SP = 0x20040000
│  0x2003FFFC │  [LEER - hier wird als ERSTES geschrieben]
│  0x2003FFF8 │  [LEER]
│  0x2003FFF4 │  [LEER]
│  0x2003FFF0 │  [LEER]
│     ...     │  [LEER - Stack wächst nach UNTEN]
│     ...     │
│  0x20000100 │  [Daten/Code/Heap]
│  0x20000000 │  [Beginn RAM]
↓
NIEDRIGE Adressen (Start des RAMs)

*/


/*
Unser Fall mit MPS und PSP Stacks:
0x2000XXXX ← msp_stack[0] (Beginn MSP-Array)
    │
    │  [4096 Bytes MSP Stack Space]
    │
0x2000YYYY ← msp_stack[4096] (Ende MSP-Array) <-- MSP zeigt hierhin
    │
0x2000ZZZZ ← psp_stack[0] (Beginn PSP-Array)
    │
    │  [2048 Bytes PSP Stack Space]
    │
0x2000WWWW ← psp_stack[2048] (Ende PSP-Array) <-- PSP zeigt hierhin

*/

// 24-Bit Countdown-Timer im Cortex-M4, löst bei 0 einen Interrupt aus und lädt automatisch neu
void systick_init(void) {
    // Reload-Wert berechnen: Timer zählt (reload+1) Takte bis zum Interrupt
    // Bei 4MHz Tacktung und SYSTICK_HZ=1000 (Interrupt-Interval): 4.000.000/1000 - 1 = 3999 
    // --> Interrupt alle 1ms mit reload=3999

    // SystemCoreClock global aus CMSIS
    uint32_t reload = (SystemCoreClock / SYSTICK_HZ) - 1u;
    
    // 1. Timer aufsetzen --> 2. Dann Timer auslösen
    // Startwert für Countdown (24-Bit, max 0xFFFFFF), wird automatisch bei 0 neu geladen
    SysTick->LOAD = reload;
    // Val speichert aktuellen Zählerstand (Reihenfolge: Reload, reload-1, ..., 1, 0)
    // Zähler zurücksetzen und (Countflag in CTRL wird gelöscht)
    SysTick->VAL  = 0u;
    
    // Nested Vectored Interrupt Controller (NVIC) 
    // Niedrigste Priorität (15), damit UART-IRQ Vorrang hat (Cortex-M4 4bits für Prio)
    NVIC_SetPriority(SysTick_IRQn, 15);
    
    // Timer aktivieren: Bits 2-0 werden gesetzt: CLKSOURCE: | Tickint: Interrupt-Request an NVIC bei 0 | Enable (Timer aktiv)
    // Control and Status Register
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
}

void init_stacks(void) {
    // MSP auf Ende des MSP-Stack-Bereichs setzen
    uint32_t msp_top = (uint32_t)&msp_stack[MSP_STACK_SIZE];
    // CMSIS-Funktion zum Setzen des MSP
    __set_MSP(msp_top);

    // PSP auf Ende des PSP-Stack-Bereichs setzen
    uint32_t psp_top = (uint32_t)&psp_stack[PSP_STACK_SIZE];
    // CMSIS-Funktion zum Setzen des PSP
    __set_PSP(psp_top);

    // CONTROL Register setzen: PSP für Thread Mode verwenden
    // 3 relevante Bits: Bit 0: nPRIV (0=privileged, 1=unprivileged), Bit 1: SPSEL (0=MSP, 1=PSP), 
    // Bit 2: FPCA (Floating Point Context Active)
    // SPSEL = 1 (PSP aktiv)
    __set_CONTROL(__get_CONTROL() | 0x02);

    // Instruction Synchronization Barrier einfügen, um sicherzustellen, dass die Änderung wirksam wird
    // Cache leeren/invalidieren, damit garantiert wird, dass die nächsten Instruktionen mit dem neuen Stack ausgeführt werden
    __ISB();
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

// SysTick ISR
// Wird bei jedem Timer-underflow aufgerufen. ISR kurz halten --> nur Flags setzen
void SysTick_Handler(void) {
    // Systemzeit erhöhen (für delay_ms) (4Mhz gtickes in 1 Sekunde)
    // nach 17,8833 Minuten überläuft der 32-Bit Zähler (2^32 / 4.000.000)
    g_ticks++;
    
    // Frequenzteiler für Timer-Events
    // static, damit der Wert zwischen den Aufrufen erhalten bleibt (Initialisierung nur einmal bei Programmstart)
    static uint32_t div = 0;
    // Alle (SYSTICK_HZ / TIMER_EVENT_HZ) Aufrufe ein Timer-Event auslösen
    // --> bei 1000Hz / 8Hz = alle 125 Aufrufe (also alle 125ms)
    if (++div >= (SYSTICK_HZ / TIMER_EVENT_HZ)) {
        div = 0;

        // Flag setzen (wird in os_poll in main.c ausgewertet)
        // Main-Loop gibt "!" aus (nicht hier, wegen Laufzeit)
        g_timer_event = 1;
    }
}
