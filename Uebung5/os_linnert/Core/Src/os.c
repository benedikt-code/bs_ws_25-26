#include "os.h"
#include "stm32l4xx.h"
#include "uart.h"

typedef struct
{
    uint32_t *sp;      // gespeicherter PSP (zeigt auf das gesicherte R4..R11-Fenster)
    t_state_t state;
} tcb_t;

#define OS_IDLE_INDEX   (OS_MAX_USER_THREADS)

static tcb_t     g_tcb[OS_MAX_USER_THREADS + 1];
static uint32_t  g_stacks[OS_MAX_USER_THREADS + 1][OS_STACK_WORDS] __attribute__((aligned(8)));

static volatile int32_t  g_current = -1;      // -1 heißt: Bootstrap/noch kein Thread aktiv
static volatile uint32_t g_time_ms = 0;

// Wartender Thread für getchar (Index des TCB), -1 = keiner
static volatile int32_t g_waiting_for_input = -1;

// Bootstrap-PSP-Stack, damit die erste PendSV nicht direkt einen frischen Thread-Stack zerschießt
static uint32_t g_boot_psp_stack[128] __attribute__((aligned(8)));

static uint32_t* prepare_stack(uint32_t *stack_top, os_thread_fn_t fn, void *arg) {
    uintptr_t top = (uintptr_t)stack_top;
    top &= ~(uintptr_t)0x7u;  // auf 8-Byte ausrichten, sonst meckert der Cortex
    uint32_t *sp = (uint32_t*)top;

    // Hardware-speicherrahmen (8 Worte): R0 R1 R2 R3 R12 LR PC xPSR
    *(--sp) = 0x01000000u;                 // xPSR (Thumb-Bit gesetzt)
    *(--sp) = (uint32_t)(uintptr_t)fn;     // PC (Startadresse vom Thread)
    *(--sp) = (uint32_t)(uintptr_t)os_thread_exit; // LR, falls der Thread frech zurückkehrt
    *(--sp) = 0u;                          // R12
    *(--sp) = 0u;                          // R3
    *(--sp) = 0u;                          // R2
    *(--sp) = 0u;                          // R1
    *(--sp) = (uint32_t)(uintptr_t)arg;    // R0 (Argument reinstecken)

    // Platz für PendSV-Handsave/-restore: R4..R11 (8 Worte)
    sp -= 8;
    for (int i = 0; i < 8; i++) sp[i] = 0u;

    return sp; // zeigt jetzt genau auf den R4..R11-Block
}

static void idle_thread(void *arg) {
    (void)arg;
    for (;;) {
        __WFI();
    }
}

// Callback vom UART-Driver (wird in os_init registriert)
static void uart_rx_cb(uint8_t b) {
    int32_t waiter = g_waiting_for_input;
    if (waiter >= 0 && waiter < (int32_t)OS_MAX_USER_THREADS) {
        uint32_t *sp = g_tcb[waiter].sp;
        if (sp) {
            uint32_t *hwframe = sp - 8; // Hardware-stacked frame liegt oberhalb des R4..R11-Blocks
            hwframe[0] = (uint32_t)b;  // R0 = Rückgabewert für getchar
        }
        g_waiting_for_input = -1;
        g_tcb[waiter].state = T_READY;
        // Kontextwechsel anfordern, damit der geweckte Thread ggf. laufen kann
        SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
        __DSB(); __ISB();
    }
}

static int pick_next(void) {
    // Round-Robin über READY-User-Threads, ansonsten geht's in den Idle
    int start = (g_current < 0) ? 0 : (int)((g_current + 1) % (int)(OS_MAX_USER_THREADS + 1));

    for (unsigned i = 0; i < OS_MAX_USER_THREADS; i++) {
        int idx = (start + (int)i) % (int)(OS_MAX_USER_THREADS);
        if (g_tcb[idx].state == T_READY) return idx;
    }

    return OS_IDLE_INDEX;
}

static int alloc_user_slot(void) {
    for (unsigned i = 0; i < OS_MAX_USER_THREADS; i++) {
        if (g_tcb[i].state == T_UNUSED || g_tcb[i].state == T_ZOMBIE) {
            return (int)i;
        }
    }
    return -1;
}

void os_init(void) {
    NVIC_SetPriority(PendSV_IRQn, 15);
    NVIC_SetPriority(SysTick_IRQn, 14);

    for (unsigned i = 0; i < (OS_MAX_USER_THREADS + 1); i++) {
        g_tcb[i].sp = 0;
        g_tcb[i].state = T_UNUSED;
    }

    // Idle-Thread zum Chillen, falls keiner was zu tun hat
    uint32_t *top = &g_stacks[OS_IDLE_INDEX][OS_STACK_WORDS];
    g_tcb[OS_IDLE_INDEX].sp = prepare_stack(top, idle_thread, 0);
    g_tcb[OS_IDLE_INDEX].state = T_READY;

    g_current = -1;
    g_time_ms = 0;

    // Kernel-Callback für UART empfangene Bytes registrieren
    // Wenn ein Thread auf getchar() blockiert, wird er hierauf aufgeweckt
    uart2_set_rx_callback(uart_rx_cb);
}

int os_thread_create(os_thread_fn_t fn, void *arg) {
    __disable_irq();
    int slot = alloc_user_slot();
    if (slot >= 0) {
        uint32_t *top = &g_stacks[slot][OS_STACK_WORDS];
        g_tcb[slot].sp = prepare_stack(top, fn, arg);
        g_tcb[slot].state = T_READY;
    }
    __enable_irq();
    return slot;
}

int os_thread_create_from_isr(os_thread_fn_t fn, void *arg) {
    int slot = alloc_user_slot();
    if (slot >= 0) {
        uint32_t *top = &g_stacks[slot][OS_STACK_WORDS];
        g_tcb[slot].sp = prepare_stack(top, fn, arg);
        g_tcb[slot].state = T_READY;
    }
    return slot;
}

void os_yield(void) {
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    __DSB(); __ISB();
}

void os_thread_exit(void) {
    __disable_irq();
    int cur = (int)g_current;
    if (cur >= 0 && cur < (int)OS_MAX_USER_THREADS) {
        g_tcb[cur].state = T_ZOMBIE;
    }
    __enable_irq();

    os_yield();
    for (;;) __WFI();
}

uint32_t os_time_ms(void) {
    return g_time_ms;
}

void os_sleep_ms(uint32_t ms) {
    // Blockierendes Sleep: TCB wird in T_BLOCKED_SLEEP geschoben und beim Tick geweckt
    os_k_sleep_blocking(ms);
}

void os_tick_isr(void) {
    g_time_ms += OS_TIME_SLICE_MS;

    // Wake up sleeping threads whose wakeup_ms <= now
    for (unsigned i = 0; i < OS_MAX_USER_THREADS; i++) {
        if (g_tcb[i].state == T_BLOCKED_SLEEP) {
            if ((int32_t)(g_time_ms - g_tcb[i].wakeup_ms) >= 0) {
                g_tcb[i].state = T_READY;
            }
        }
    }

    // request context switch
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

// --- Kernel-side helpers invoked from SVC dispatcher ---
int os_k_getchar_blocking(void) {
    int c = uart2_getc_nonblocking();
    if (c != -1) return c;

    __disable_irq();
    int cur = (int)g_current;
    if (cur >= 0 && cur < (int)OS_MAX_USER_THREADS) {
        g_tcb[cur].state = T_BLOCKED_IO;
        g_waiting_for_input = cur;
    }
    __enable_irq();

    // cause scheduler to run
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    __DSB(); __ISB();

    return -1; // actual return value will be injected by IRQ handler when data arrives
}

void os_k_sleep_blocking(uint32_t ms) {
    __disable_irq();
    int cur = (int)g_current;
    if (cur >= 0 && cur < (int)OS_MAX_USER_THREADS) {
        g_tcb[cur].state = T_BLOCKED_SLEEP;
        g_tcb[cur].wakeup_ms = g_time_ms + ms;
    }
    __enable_irq();

    // request context switch
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    __DSB(); __ISB();
}

void os_start(void) {
    // Thread-Mode nutzt PSP, aber wir starten mit dem BOOT-PSP, damit die erste PendSV keinen echten Stack zerlegt
    __set_PSP((uint32_t)(g_boot_psp_stack + (sizeof(g_boot_psp_stack)/sizeof(g_boot_psp_stack[0]))));
    __set_CONTROL((__get_CONTROL() & ~1u) | 2u); // privileged + PSP
    __ISB();

    g_current = -1;

    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    __DSB(); __ISB();

    for (;;) __WFI();
}

// ---- called from PendSV_Handler (assembly) ----
uint32_t* os_pendsv_schedule(uint32_t *cur_sp) {
    int cur = (int)g_current;

    // Save current thread if there is one
    if (cur >= 0) {
        if (g_tcb[cur].state == T_RUNNING) {
            g_tcb[cur].state = T_READY;
        }
        g_tcb[cur].sp = cur_sp;
    }

    int next = pick_next();

    // Vorgabe: Bei jedem echten Context-Switch (also wenn Thread wechselt) einmal \r\n ausgeben
    if (next != cur && cur >= 0) {
        uart2_putc('\r');
        uart2_putc('\n');
    }

    g_current = next;
    g_tcb[next].state = T_RUNNING;

    return g_tcb[next].sp;
}
