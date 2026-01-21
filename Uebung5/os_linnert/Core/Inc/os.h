#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===== Config =====
#define OS_MAX_USER_THREADS   16u        
#define OS_STACK_WORDS        256u       // 256 words = 1024 bytes pro thread stack
#define OS_TIME_SLICE_MS      50u        // timeslice (SysTick interval)

// ===== Types =====
typedef void (*os_thread_fn_t)(void *arg);

// ===== API =====
void     os_init(void);
void     os_start(void);                 // hier wird der OS Scheduler gestartet (blockiert forever)

int      os_thread_create(os_thread_fn_t fn, void *arg);
int      os_thread_create_from_isr(os_thread_fn_t fn, void *arg);

void     os_yield(void);
void     os_thread_exit(void) __attribute__((noreturn));

void     os_sleep_ms(uint32_t ms);

uint32_t os_time_ms(void);

// Called from SysTick_Handler
void     os_tick_isr(void);

/* SVC/SWI Aufrufkonvention (Kernel <-> Anwendung)
 * - Argumente: r0..r3
 * - Syscall-Nummer: r12
 * - Rückgabewert: r0
 * Anwendungen setzen r12 auf die gewünschte SYS_ Konstante,
 * legen Argumente in r0..r3 und führen `svc 0` aus.
 * Der SVC-Handler liest das gestackte Registerframe (r0..r3,r12,LR,PC,xPSR)
 * und dispatcht die entsprechenden Kernel-Funktionen.
 */

// Erweiterte Thread-States, werden vom Kernel beim Blockieren genutzt
typedef enum {
	T_UNUSED = 0,
	T_READY,
	T_RUNNING,
	T_ZOMBIE,
	T_BLOCKED_IO,
	T_BLOCKED_SLEEP,
} t_state_t;

// Kernel-intern: Funktionen die vom SVC-Dispatcher genutzt werden
int  os_k_getchar_blocking(void);
void os_k_sleep_blocking(uint32_t ms);

#ifdef __cplusplus
}
#endif
