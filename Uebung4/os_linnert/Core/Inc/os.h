#ifndef OS_H
#define OS_H

#include <stdint.h>

#define OS_MAX_THREADS 16
#define OS_STACK_WORDS 256

typedef void (*thread_fn_t)(void *arg);

void os_init(void);
int  os_thread_create(thread_fn_t fn, void *arg);
int  os_thread_create_from_isr(thread_fn_t fn, void *arg);
void os_start(void);
void os_yield(void);
void os_thread_exit(void);

// worker used by uart ISR for demo
void thread_echo_worker(void *arg);

#endif // OS_H
