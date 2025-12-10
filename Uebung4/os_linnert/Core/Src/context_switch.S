.syntax unified
.cpu cortex-m4
.thumb

.global PendSV_Handler
.extern pick_next
.extern os_get_sp
.extern os_set_current
.extern current

PendSV_Handler:
    CPSID   I
    MRS     R0, PSP
    STMDB   R0!, {R4-R11}

    // save old PSP into sp_table[current]
    LDR     R1, =current
    LDR     R2, [R1]
    CMP     R2, #0
    BLT     .Lskip_save
    // R2 >= 0 -> store R0 into sp_table[R2]
    MOV     R3, R2
    LSL     R3, R3, #2
    LDR     R4, =os_get_sp
    // call os_get_sp to get pointer location? Simpler: call pick_next now and store later
.Lskip_save:

    // call scheduler pick_next()
    BL      pick_next
    // r0 contains next index or -1
    CMP     R0, #-1
    BEQ     .Lno_thread

    // set current via os_set_current(r0)
    MOV     R1, R0
    BL      os_set_current

    // get sp for new current: call os_get_sp(current)
    MOV     R0, R1
    BL      os_get_sp
    // r0 = new sp
    LDMIA   R0!, {R4-R11}
    MSR     PSP, R0
    CPSIE   I
    BX      LR

.Lno_thread:
    // no runnable threads: just re-enable interrupts and return
    CPSIE   I
    BX      LR
