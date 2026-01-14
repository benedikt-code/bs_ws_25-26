.syntax unified
.thumb

.global PendSV_Handler
.type PendSV_Handler, %function

// extern uint32_t* os_pendsv_schedule(uint32_t *cur_sp);
.extern os_pendsv_schedule

PendSV_Handler:
    // Get PSP (thread stack)
    mrs     r0, psp

    // Save callee-saved regs on PSP: r4-r11
    stmdb   r0!, {r4-r11}

    // Save EXC_RETURN across BL
    push    {lr}

    // r0 = current saved PSP; returns r0 = next thread saved PSP
    bl      os_pendsv_schedule

    // Restore EXC_RETURN
    pop     {lr}

    // Restore r4-r11 from next thread PSP
    ldmia   r0!, {r4-r11}

    // Write PSP back
    msr     psp, r0

    // Exception return (uses PSP, returns to thread mode)
    bx      lr
