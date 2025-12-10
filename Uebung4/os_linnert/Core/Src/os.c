#include "os.h"
#include "stm32l4xx.h"
#include "uart.h"

typedef enum { THREAD_FREE = 0, THREAD_READY, THREAD_RUNNING, THREAD_DEAD } thread_state_t;

static uint32_t *sp_table[OS_MAX_THREADS];      // Stack pointers for each thread
static thread_state_t state_table[OS_MAX_THREADS];      // State of each thread (Free, Ready, Running, Dead)
static uint32_t stacks[OS_MAX_THREADS][OS_STACK_WORDS]; // Stacks for each thread
static int current = -1;        // Index of current running thread, -1 if none
static int thread_count = 0;    // #Threads

// Default xPSR value with Thumb bit set
// Hier stehen APSR (condition flags), EPSR (execution state), IPSR (exception number) drinne
static const uint32_t initial_xpsr = 0x01000000U; 

// OS Grundstrukturen initialisieren
void os_init(void) {
    for (int i = 0; i < OS_MAX_THREADS; i++) {
        sp_table[i] = NULL;     // alle stack pointer auf NULL setzen
        state_table[i] = THREAD_FREE;     // alle threads auf FREE setzen
    }
}

static void prepare_stack(int id, thread_fn_t fn, void *arg) {
    uint32_t *sp = &stacks[id][OS_STACK_WORDS];
    // space for R4-R11 (saved by PendSV)
    sp -= 8;
    for (int i = 0; i < 8; i++) sp[i] = 0;
    // exception stack frame (xPSR, PC, LR, R12, R3, R2, R1, R0)
    sp -= 8;
    sp[0] = initial_xpsr;             // xPSR
    sp[1] = (uint32_t)fn;             // PC
    sp[2] = (uint32_t)os_thread_exit; // LR -> when thread returns
    sp[3] = 0; // R12
    sp[4] = 0; // R3
    sp[5] = 0; // R2
    sp[6] = 0; // R1
    sp[7] = (uint32_t)arg; // R0 (argument)
    sp_table[id] = sp;
}

// Neuen Thread erstellen (nicht ISR - Interrupt Service Routine)
int os_thread_create(thread_fn_t fn, void *arg) {
    __disable_irq();
    // wir gehen alle state_table durch und suchen freien slot
    for (int i = 0; i < OS_MAX_THREADS; i++) {
        if (state_table[i] == THREAD_FREE) {
            prepare_stack(i, fn, arg);
            state_table[i] = THREAD_READY;
            thread_count++;
            __enable_irq();
            return i; // return thread id und gehen damit aus for-Schleife
        }
    }
    __enable_irq();
    return -1; // kein freier slot gefunden
}

// Neuen Thread erstellen aus ISR Kontext
int os_thread_create_from_isr(thread_fn_t fn, void *arg) {
    for (int i = 0; i < OS_MAX_THREADS; i++) {
        if (state_table[i] == THREAD_FREE) {
            prepare_stack(i, fn, arg);
            state_table[i] = THREAD_READY;
            thread_count++;
            return i;
        }
    }
    return -1;
}

// scheduler helper called from PendSV (or assembly)
int pick_next(void) {
    if (thread_count == 0) return -1;
    int start = (current < 0) ? 0 : (current + 1) % OS_MAX_THREADS;
    for (int i = 0; i < OS_MAX_THREADS; i++) {
        int idx = (start + i) % OS_MAX_THREADS;
        if (state_table[idx] == THREAD_READY) return idx;
    }
    return -1;
}

// expose current for assembly via function
int os_get_current(void) {
    return current;
}

// store given sp for index
void os_store_sp(int idx, uint32_t *sp) {
    if (idx < 0 || idx >= OS_MAX_THREADS) return;
    sp_table[idx] = sp;
}

void os_yield(void) {
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}

void os_thread_exit(void) {
    __disable_irq();
    if (current >= 0) {
        state_table[current] = THREAD_DEAD;
        sp_table[current] = NULL;
        thread_count--;
    }
    // trigger context switch
    SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
    for(;;) { __WFI(); }
}

// This function is called from PendSV handler (assembly) after pick_next
// Expose helpers for assembly
int os_set_current(int idx) {
    if (idx < 0) {
        current = -1;
        return -1;
    }
    current = idx;
    state_table[current] = THREAD_RUNNING;
    return current;
}

uint32_t *os_get_sp(int idx) {
    if (idx < 0) return NULL;
    return sp_table[idx];
}

// Start the OS: set PendSV priority low and start first thread by exception return
void os_start(void) {
    // make PendSV lowest priority
    // 0 = highest, larger = lower priority. Use priority 15 (assuming 4-bit priorities)
    NVIC_SetPriority(PendSV_IRQn, 15);

    // pick first thread
    int next = pick_next();
    if (next < 0) {
        // nothing to run -> idle forever
        for(;;) { __WFI(); }
    }
    current = next;
    state_table[current] = THREAD_RUNNING;

    // set PSP to thread SP and switch to using PSP
    __set_PSP((uint32_t)sp_table[current]);
    // set CONTROL: use PSP (bit1 = 1), remain privileged (bit0 = 0)
    __set_CONTROL((__get_CONTROL() & ~1) | 2);
    __ISB();

    // perform exception return to thread (use special EXC_RETURN value)
    __asm volatile (
        "MOV LR, #0xFFFFFFFD\n" // return using PSP, Thread mode
        "BX LR\n"
    );
}

// demo worker: print received char 5 times then exit
void thread_echo_worker(void *arg) {
    char c = (char)(uintptr_t)arg;
    for (int i = 0; i < 5; i++) {
        uart2_putc(c);
        // small busy wait
        for (volatile uint32_t d = 0; d < 200000u; d++) __asm__("nop");
        os_yield();
    }
    os_thread_exit();
}
