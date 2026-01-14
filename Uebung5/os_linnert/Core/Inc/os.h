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

#ifdef __cplusplus
}
#endif
