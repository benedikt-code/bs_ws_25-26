#pragma once
#include <stdint.h>
#include "os.h"

// Syscall-Nummern
enum {
    SYS_PUTCHAR    = 1,
    SYS_GETCHAR    = 2,
    SYS_THREAD_EXIT= 3,
    SYS_THREAD_CREATE = 4,
    SYS_SLEEP_MS   = 5,
};

// Leichte Benutzer-Wrapper (inline, führen `svc 0` mit r12 = syscall)
static inline int sys_putchar(int c) {
    register int r0 asm("r0") = c;
    register int r12 asm("r12") = SYS_PUTCHAR;
    asm volatile ("svc 0" : "+r"(r0) : "r"(r12) : "memory");
    return r0;
}

static inline int sys_getchar(void) {
    register int r0 asm("r0");
    register int r12 asm("r12") = SYS_GETCHAR;
    asm volatile ("svc 0" : "=r"(r0) : "r"(r12) : "memory");
    return r0;
}

static inline int sys_thread_create(os_thread_fn_t fn, void *arg) {
    register int r0 asm("r0") = (int)fn;
    register int r1 asm("r1") = (int)arg;
    register int r12 asm("r12") = SYS_THREAD_CREATE;
    asm volatile ("svc 0" : "+r"(r0) : "r"(r1), "r"(r12) : "memory");
    return r0;
}

static inline void sys_thread_exit(void) {
    register int r12 asm("r12") = SYS_THREAD_EXIT;
    asm volatile ("svc 0" : : "r"(r12) : "memory");
}

static inline void sys_sleep_ms(uint32_t ms) {
    register uint32_t r0 asm("r0") = ms;
    register int r12 asm("r12") = SYS_SLEEP_MS;
    asm volatile ("svc 0" : "+r"(r0) : "r"(r12) : "memory");
}
